#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include <Eigen/Dense>

#include <cstddef>
#include <functional>

using namespace std::chrono_literals;

class DistanceController : public rclcpp::Node {

public:
  DistanceController()
      : Node("distance_controller_node"), scene_number_(scene_number) {
    using std::placeholders::_1;

    // Stable PID values: kP=0.3; kD=1.0; kI=0.01
    kP_ = this->declare_parameter<float>("kP", 0.6f);
    kI_ = this->declare_parameter<float>("kI", 0.01f);
    kD_ = this->declare_parameter<float>("kD", 1.1f);

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
        "odometry/filtered", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          //   current_pose_(0) = msg->pose.pose.position.x;
          //   current_pose_(1) = msg->pose.pose.position.y;
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
    switch (scene_number_) {
    case 1: // Simulation
      // Assign waypoints for Simulation
      break;
    case 2: // CyberWorld
      // Assign waypoints for CyberWorld
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid Scene Number: %d",
                   scene_number_);
    }
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
        float max_vel = 2.5f;
        if (err_pose.norm() > 0.35f && input.norm() > max_vel) {
          input = input.normalized() * max_vel;
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "vel: x=%.3f y=%.3f | err=%.3f", cmd_vel.linear.x,
                             cmd_vel.linear.y, err_pose.norm());

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

  Eigen::Vector2f current_pose_;
  std::vector<Eigen::Vector2f> waypoints_{

      {0.0, 1.0},  //  1
      {0.0, 0.0},  //  Home
      {0.0, -1.0}, //  2
      {0.0, 0.0},  //  Home
      {1.0, 1.0},  //  3
      {0.0, 0.0},  //  Home
      {1.0, -1.0}, //  4
      {0.0, 0.0},  //  Home
      {1.0, 0.0},  //  5
      {0.0, 0.0},  //  Home

  };

  int scene_number_;

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