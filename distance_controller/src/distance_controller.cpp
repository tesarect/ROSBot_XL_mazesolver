#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <nlohmann/json.hpp>

#include <cstddef>
#include <functional>
#include <string>

using namespace std::chrono_literals;

class DistanceController : public rclcpp::Node {

public:
  DistanceController(int scene_number)
      : Node("distance_controller_node"), scene_number_(scene_number) {
    using std::placeholders::_1;

    // Waypoints selection to perform the sequence.
    // for all waypoints to be executed, -1 = all waypoints
    // default selection is 2 from 0(home position)
    num_waypoints_ = this->declare_parameter<int>("num_waypoints", 3);
    odom_topic_ = this->declare_parameter<std::string>("odom_topic",
                                                       "/odometry/filtered");

    // Stable PID values: kP=0.3; kD=1.0; kI=0.01
    if (scene_number_ == 1) {
      kP_ = this->declare_parameter<float>("kP", 0.6f);
      kI_ = this->declare_parameter<float>("kI", 0.01f);
      kD_ = this->declare_parameter<float>("kD", 1.1f);
    } else {
      kP_ = this->declare_parameter<float>("kP", 0.3f);
      kI_ = this->declare_parameter<float>("kI", 0.01f);
      kD_ = this->declare_parameter<float>("kD", 1.0f);
    };

    SelectWaypoints();

    // Timer fires once after 1 s to let odom settle, then runs control loop
    timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = this->create_wall_timer(
        1s, std::bind(&DistanceController::timer_callback, this),
        timer_callback_group_);

    // Odometry subscriber
    odom_sub_options_.callback_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          current_pose_ << msg->pose.pose.position.x, msg->pose.pose.position.y;
        },
        odom_sub_options_);

    // Velocity publisher
    twist_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    current_pose_ << 0.0f, 0.0f;
    RCLCPP_INFO(this->get_logger(), "PID gains: kP=%.3f kI=%.3f kD=%.3f", kP_,
                kI_, kD_);
    RCLCPP_INFO(this->get_logger(),
                "Distance controller initialization completed.");
  }

private:
  // Waypoint selection based on scene selection
  void SelectWaypoints() {
    std::string file_path;
    switch (scene_number_) {
    case 1: // Simulation
            // Assign waypoints for Simulation
      file_path = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/resources/"
                  "waypoints/sim_waypoints.json";
      break;
    case 2: // CyberWorld
      // Assign waypoints for CyberWorld
      file_path = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/resources/"
                  "waypoints/real_waypoints.json";
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid Scene Number: %d",
                   scene_number_);
      return;
    }

    // Load JSON file
    std::ifstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open waypoints file: %s",
                   file_path.c_str());
      return;
    }

    nlohmann::json json_data;
    file >> json_data;

    // Parse all waypoints from JSON
    // std::vector<Eigen::Vector2f> all_waypoints;
    for (auto &wp : json_data) {
      float x = wp["data"][odom_topic_]["position"]["x"];
      float y = wp["data"][odom_topic_]["position"]["y"];
      all_waypoints.push_back({x, y});
    }
    RCLCPP_INFO(this->get_logger(), "Total %zu waypoints loaded from json file",
                all_waypoints.size());

    std::ostringstream oss_1;
    oss_1 << "All waypoints loaded: [";
    for (size_t i = 0; i < all_waypoints.size(); ++i) {
      oss_1 << "[" << all_waypoints[i].x() << ", " << all_waypoints[i].y()
            << "]";
      if (i != all_waypoints.size() - 1)
        oss_1 << ", ";
    }
    oss_1 << "]";

    RCLCPP_DEBUG(this->get_logger(), "%s", oss_1.str().c_str());

    // Apply num_waypoints limit (0 to N inclusive)
    // int limit =
    //     (num_waypoints_ == -1) ? (int)all_waypoints.size() - 1 :
    //     num_waypoints_;
    // limit = std::min(limit, (int)all_waypoints.size() - 1);
    // int limit =
    //     (num_waypoints_ == -1) ? (int)all_waypoints.size() : num_waypoints_;
    // limit = std::min(limit, (int)all_waypoints.size());
    int limit = (num_waypoints_ == -1)
                    ? (int)all_waypoints.size()
                    : std::min(num_waypoints_, (int)all_waypoints.size());

    // Build forward slice: index 0 to limit
    // std::vector<Eigen::Vector2f> forward(all_waypoints.begin(),
    //                                      all_waypoints.begin() + limit + 1);
    std::vector<Eigen::Vector2f> forward(all_waypoints.begin(),
                                         all_waypoints.begin() + limit);

    // Build return path: reverse of forward, skipping the last point (avoid
    // duplicate)
    std::vector<Eigen::Vector2f> reverse_path(forward.rbegin() + 1,
                                              forward.rend());
    // std::vector<Eigen::Vector2f> reverse_path(forward.rbegin(),
    // forward.rend());

    // Combine: forward <-and-> return
    waypoints_ = forward;
    waypoints_.insert(waypoints_.end(), reverse_path.begin(),
                      reverse_path.end());

    std::ostringstream oss_2;
    oss_2 << "waypoints selected: [";
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      oss_2 << "[" << waypoints_[i].x() << ", " << waypoints_[i].y() << "]";
      if (i != waypoints_.size() - 1)
        oss_2 << ", ";
    }
    oss_2 << "]";
    RCLCPP_INFO(this->get_logger(), "%s", oss_2.str().c_str());

    // RCLCPP_INFO(this->get_logger(),
    //             "Loaded %zu waypoints (forward=%zu + return=%zu) | limit=%d",
    //             waypoints_.size(), forward.size(), reverse_path.size(),
    //             limit);
  }

  // Control loop
  void timer_callback() {
    timer_->cancel();

    rclcpp::Rate rate(100ms); // 10 Hz control loop
    geometry_msgs::msg::Twist cmd_vel;

    Eigen::Vector2f err_pose;
    Eigen::Vector2f prev_pose{0.0f, 0.0f};
    Eigen::Vector2f sum_I;
    Eigen::Vector2f X_dot; // robot velocity estimate (Delta pose/Delta t)
    Eigen::Vector2f input; // PID output → velocity command

    rclcpp::Time current_time = this->get_clock()->now();
    rclcpp::Time prev_time = this->get_clock()->now();
    float dt;

    for (size_t i = 0; i < waypoints_.size(); i++) {

      RCLCPP_INFO(this->get_logger(), "Moving to waypoint %zu", i + 1);

      // Reset per-waypoint PID state
      err_pose = waypoints_[i] - current_pose_;
      sum_I << 0.0f, 0.0f;
      X_dot << 0.0f, 0.0f;
      prev_pose = current_pose_;
      prev_time = this->get_clock()->now();

      // PID loop until goal is within tolerance
      while (err_pose.norm() >= 0.02f && rclcpp::ok()) {

        err_pose = waypoints_[i] - current_pose_;
        current_time = this->get_clock()->now();
        dt = (current_time - prev_time).seconds();

        if (dt <= 0.0f) {
          rate.sleep();
          continue;
        }

        // Integral term (vector)
        sum_I += err_pose * dt;

        // Derivative term: robot velocity (opposes motion → damping)
        X_dot = (current_pose_ - prev_pose) / dt;

        // PID output
        //   P: proportional to remaining distance
        //   I: accumulates steady-state error
        //   D: damps velocity (reduces overshoot)
        input = kP_ * err_pose + kI_ * sum_I + kD_ * X_dot;

        if (err_pose.norm() > dist_threshold && input.norm() > max_vel) {
          input = input.normalized() * max_vel;
        }
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        //                      "vel: x=%.3f y=%.3f | err=%.3f",
        //                      cmd_vel.linear.x, cmd_vel.linear.y,
        //                      err_pose.norm());

        cmd_vel.linear.x = input(0);
        cmd_vel.linear.y = input(1);
        twist_pub_->publish(cmd_vel);

        RCLCPP_DEBUG(
            this->get_logger(),
            "err=(%.3f, %.3f) P=(%.3f,%.3f) I=(%.3f,%.3f) D=(%.3f,%.3f)",
            err_pose(0), err_pose(1), kP_ * err_pose(0), kP_ * err_pose(1),
            kI_ * sum_I(0), kI_ * sum_I(1), kD_ * X_dot(0), kD_ * X_dot(1));

        prev_pose = current_pose_;
        prev_time = current_time;
        rate.sleep();
      }

      RCLCPP_INFO(this->get_logger(), "Waypoint %zu reached.", i + 1);

      // Stop and pause before moving to next waypoint
      cmd_vel.linear.x = 0.0f;
      cmd_vel.linear.y = 0.0f;
      twist_pub_->publish(cmd_vel);
      rclcpp::sleep_for(1s);
    }

    // Final stop
    cmd_vel.linear.x = 0.0f;
    cmd_vel.linear.y = 0.0f;
    twist_pub_->publish(cmd_vel);
    RCLCPP_INFO(this->get_logger(), "Waypoints complete.");
  }

  float kP_; // Proportional
  float kI_; // Integral
  float kD_; // Derivative

  float max_vel = 3.5f;
  float dist_threshold = 0.45f;

  Eigen::Vector2f current_pose_;
  std::vector<Eigen::Vector2f> waypoints_;
  std::vector<Eigen::Vector2f> all_waypoints;

  int scene_number_;
  int num_waypoints_;
  std::string odom_topic_;

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::SubscriptionOptions odom_sub_options_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  int scene_number = 1;
  if (argc > 1) {
    scene_number = std::atoi(argv[1]);
  }
  auto node = std::make_shared<DistanceController>(scene_number);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}