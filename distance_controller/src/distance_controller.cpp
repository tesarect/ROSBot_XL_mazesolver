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
#include <sstream>
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
      goal_tolerance_ = this->declare_parameter<float>("goal_tol", 0.02f);
    } else {
      // slow and stable :  kP:=0.090; kD:=0.3; kI:=0.001
      // normal speed : kP:=0.25; kD:=0.23; kI:=0.001
      //   kP:=0.27; -p kD:=0.23; -p kI:=0.001
      kP_ = this->declare_parameter<float>("kP", 0.27f);
      kI_ = this->declare_parameter<float>("kI", 0.001f);
      kD_ = this->declare_parameter<float>("kD", 0.23f);
      goal_tolerance_ = this->declare_parameter<float>("goal_tol", 0.05f);
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
          // pose
          current_pose_ << msg->pose.pose.position.x, msg->pose.pose.position.y;
          // yaw
          current_yaw_qz_ = msg->pose.pose.orientation.z;
          current_yaw_qw_ = msg->pose.pose.orientation.w;
          initial_odom_received_ = true;
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
    for (auto &wp : json_data) {
      float x = wp["data"][odom_topic_]["position"]["x"];
      float y = wp["data"][odom_topic_]["position"]["y"];
      all_waypoints.push_back({x, y});
    }
    RCLCPP_INFO(this->get_logger(), "Total %zu waypoints loaded from json file",
                all_waypoints.size());

    RCLCPP_INFO(this->get_logger(), "All Waypoints: %s",
                prnt_v_eigen(all_waypoints).c_str());

    // Apply num_waypoints limit
    int limit = (num_waypoints_ == -1)
                    ? (int)all_waypoints.size()
                    : std::min(num_waypoints_, (int)all_waypoints.size());

    // Build forward slice: index 0 to limit
    std::vector<Eigen::Vector2f> forward(all_waypoints.begin(),
                                         all_waypoints.begin() + limit);

    // Build return path: reverse of forward, skipping the last point (avoid
    // duplicate)
    std::vector<Eigen::Vector2f> reverse_path(forward.rbegin() + 1,
                                              forward.rend());

    // Combine: forward <-and-> return
    waypoints_ = forward;
    waypoints_.insert(waypoints_.end(), reverse_path.begin(),
                      reverse_path.end());

    RCLCPP_INFO(this->get_logger(), "Selected Waypoints: %s",
                prnt_v_eigen(waypoints_).c_str());

    // RCLCPP_INFO(this->get_logger(),
    //             "Loaded %zu waypoints (forward=%zu + return=%zu) | limit=%d",
    //             waypoints_.size(), forward.size(), reverse_path.size(),
    //             limit);
  }

  void waypoints_delta_correction() {
    // Pre-compute deltas between consecutive JSON waypoints
    relative_deltas_.clear();
    for (size_t i = 0; i + 1 < all_waypoints.size(); i++) {
      relative_deltas_.push_back(all_waypoints[i + 1] - all_waypoints[i]);
    }

    RCLCPP_INFO(this->get_logger(), "relative deltas: %s",
                prnt_v_eigen(relative_deltas_).c_str());

    // Drift check
    Eigen::Vector2f json_home = all_waypoints[0];
    float drift = (json_home - current_pose_).norm();

    RCLCPP_INFO(this->get_logger(),
                "JSON home=(%.4f, %.4f) | live=(%.4f, %.4f) | drift=%.4f m",
                json_home.x(), json_home.y(), current_pose_.x(),
                current_pose_.y(), drift);

    if (drift > ODOM_DRIFT_THRESHOLD) {
      relative_mode_ = true;
      RCLCPP_WARN(this->get_logger(),
                  "Drift %.4f m > threshold → RELATIVE mode", drift);

      // Rebuild waypoints_ anchored to live pose using deltas
      int limit = (num_waypoints_ == -1)
                      ? (int)all_waypoints.size()
                      : std::min(num_waypoints_, (int)all_waypoints.size());

      waypoints_.clear();
      waypoints_.push_back(current_pose_); // home = live pose

      for (int i = 0; i < limit - 1; i++) {
        // yaw_offset = 0 (robot always faces forward)
        waypoints_.push_back(waypoints_.back() + relative_deltas_[i]);
      }

      RCLCPP_INFO(this->get_logger(), "Relative Waypoints: %s",
                  prnt_v_eigen(waypoints_).c_str());

      // Return path
      std::vector<Eigen::Vector2f> reverse_path(waypoints_.rbegin() + 1,
                                                waypoints_.rend());
      waypoints_.insert(waypoints_.end(), reverse_path.begin(),
                        reverse_path.end());

      RCLCPP_INFO(this->get_logger(),
                  "RELATIVE mode: %zu waypoints built from live pose",
                  waypoints_.size());
    } else {
      relative_mode_ = false;
      RCLCPP_INFO(this->get_logger(),
                  "ABSOLUTE mode: using JSON positions directly");
    }

    RCLCPP_INFO(this->get_logger(), "Final Waypoints: %s",
                prnt_v_eigen(waypoints_).c_str());
  }

  // Control loop
  void timer_callback() {
    timer_->cancel();

    // Wait for real odom reading
    rclcpp::Rate wait_rate(10ms);
    while (!initial_odom_received_ && rclcpp::ok()) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                           "Waiting for odometry...");
      wait_rate.sleep();
    }

    RCLCPP_INFO(this->get_logger(), "Live odom ready: (%.4f, %.4f)",
                current_pose_.x(), current_pose_.y());

    // Odom Yaw correction
    if (scene_number_ == 2) {
      float startup_yaw = 2.0f * std::atan2(current_yaw_qz_, current_yaw_qw_);

      // Normalize
      while (startup_yaw > M_PI)
        startup_yaw -= 2.0f * M_PI;
      while (startup_yaw < -M_PI)
        startup_yaw += 2.0f * M_PI;

      RCLCPP_INFO(this->get_logger(), "Startup yaw=%.4f rad (%.1f deg)",
                  startup_yaw, startup_yaw * 180.0f / M_PI);

      if (std::abs(startup_yaw) > 0.087f) { // > 5 degrees
        RCLCPP_WARN(this->get_logger(),
                    "Odom rotated %.1f deg → correcting deltas",
                    startup_yaw * 180.0f / M_PI);

        float cos_corr = std::cos(-startup_yaw);
        float sin_corr = std::sin(-startup_yaw);

        for (auto &delta : relative_deltas_) {
          float dx = delta(0);
          float dy = delta(1);
          delta(0) = dx * cos_corr - dy * sin_corr;
          delta(1) = dx * sin_corr + dy * cos_corr;
        }
      }
    }

    waypoints_delta_correction();

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
      //   while (err_pose.norm() >= 0.02f && rclcpp::ok()) {
      while (err_pose.norm() >= goal_tolerance_ && rclcpp::ok()) {

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

        // Increased acceleration in simulation
        if (scene_number_ == 1) {
          if (err_pose.norm() > dist_threshold && input.norm() > max_vel) {
            input = input.normalized() * max_vel;
          }
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

  template <typename T> std::string prnt_vector(const std::vector<T> &vec) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
      oss << vec[i];
      if (i != vec.size() - 1)
        oss << ", ";
    }
    oss << "]";
    return oss.str();
  }

  template <typename Derived>
  std::string prnt_eigen_v(const Eigen::MatrixBase<Derived> &mat) {
    std::ostringstream oss;
    oss << mat.transpose(); // nicer for column vectors
    return oss.str();
  }

  std::string prnt_v_eigen(const std::vector<Eigen::Vector2f> &vec) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
      oss << "(" << vec[i].x() << ", " << vec[i].y() << ")";
      if (i != vec.size() - 1)
        oss << ", ";
    }
    oss << "]";
    return oss.str();
  }

  float kP_; // Proportional
  float kI_; // Integral
  float kD_; // Derivative

  float goal_tolerance_;
  float max_vel = 3.5f;
  float dist_threshold = 0.45f;
  bool relative_mode_ = false;
  std::vector<Eigen::Vector2f> relative_deltas_;
  bool initial_odom_received_ = false;
  static constexpr float ODOM_DRIFT_THRESHOLD = 0.05f;
  float current_yaw_qz_ = 0.0f;
  float current_yaw_qw_ = 1.0f;

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