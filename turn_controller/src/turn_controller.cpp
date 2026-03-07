#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

class TurnController : public rclcpp::Node {

public:
  TurnController(int scene_number)
      : Node("turn_controller_node"), scene_number_(scene_number) {

    odom_topic_ = this->declare_parameter<std::string>("odom_topic",
                                                       "/odometry/filtered");

    if (scene_number_ == 1) {
      kP_ = this->declare_parameter<double>("kP", 3.0);
      kI_ = this->declare_parameter<double>("kI", 0.001);
      kD_ = this->declare_parameter<double>("kD", 0.5);
      yaw_tolerance_ = this->declare_parameter<double>("yaw_tol", 0.02);
      max_ang_vel_ = this->declare_parameter<double>("max_ang_vel", 1.5);
    } else {
      kP_ = this->declare_parameter<double>("kP", 0.8);
      kI_ = this->declare_parameter<double>("kI", 0.005);
      kD_ = this->declare_parameter<double>("kD", 0.2);
      yaw_tolerance_ = this->declare_parameter<double>("yaw_tol", 0.02);
      max_ang_vel_ = this->declare_parameter<double>("max_ang_vel", 0.5);
      // TODO: increase angular vel and then carry on a test
    }

    SelectWaypoints();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          tf2::Quaternion q(
              msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
              msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
          double roll, pitch, yaw;
          tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
          current_yaw_ = yaw;

          if (!odom_ready_) {
            home_yaw_ = yaw;
            start_yaw_ = yaw; // anchor first turn to live startup yaw
            odom_ready_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "Odom ready — startup yaw: %.4f rad (%.2f deg)",
                        start_yaw_, start_yaw_ * 180.0 / M_PI);
          }
        });

    twist_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    // Control loop at 10 Hz
    timer_ = this->create_wall_timer(
        100ms, std::bind(&TurnController::control_loop, this));

    RCLCPP_INFO(this->get_logger(),
                "PID: kP=%.3f kI=%.3f kD=%.3f | "
                "max_ang_vel=%.2f rad/s | yaw_tol=%.4f rad",
                kP_, kI_, kD_, max_ang_vel_, yaw_tolerance_);
    RCLCPP_INFO(this->get_logger(), "Turn controller ready.");
  }

private:
  // LoadFromYaml
  void LoadFromYaml(const std::string &file_path) {
    RCLCPP_INFO(this->get_logger(), "Loading waypoints from: %s",
                file_path.c_str());

    YAML::Node config;
    try {
      config = YAML::LoadFile(file_path);
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load YAML: %s", e.what());
      return;
    }

    if (!config["waypoints"] || !config["waypoints"].IsSequence()) {
      RCLCPP_ERROR(this->get_logger(), "YAML missing 'waypoints' sequence.");
      return;
    }

    for (size_t i = 0; i < config["waypoints"].size(); i++) {
      const auto &wp = config["waypoints"][i];
      if (!wp["yaw"]) {
        RCLCPP_WARN(this->get_logger(), "Waypoint %zu missing yaw — skipping.",
                    i);
        continue;
      }

      double yaw_delta = wp["yaw"].as<double>();
      relative_yaws_.push_back(yaw_delta);

      RCLCPP_INFO(this->get_logger(), "  WP[%zu]: dyaw=%.4f rad (%.2f deg)", i,
                  yaw_delta, yaw_delta * 180.0 / M_PI);
    }

    RCLCPP_INFO(this->get_logger(), "%zu yaw waypoints loaded.",
                relative_yaws_.size());
  }

  void SelectWaypoints() {
    std::string file_path;
    switch (scene_number_) {
    case 1:
      file_path = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/resources/"
                  "waypoints/sim_waypoints_t.yaml";
      break;
    case 2:
      file_path = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/resources/"
                  "waypoints/real_waypoints_t.yaml";
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid scene number: %d",
                   scene_number_);
      return;
    }

    LoadFromYaml(file_path);

    if (relative_yaws_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No yaw waypoints loaded — aborting.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Yaw deltas: %s", prnt_yaws().c_str());
  }

  void control_loop() {
    if (!odom_ready_)
      return;
    if (current_goal_idx_ >= relative_yaws_.size())
      return;

    if (!goal_active_) {
      target_yaw_ =
          normalize_angle(start_yaw_ + relative_yaws_[current_goal_idx_]);
      goal_active_ = true;

      RCLCPP_INFO(this->get_logger(),
                  "Turn %zu/%zu | start_yaw=%.4f rad (%.2f deg) "
                  "+ delta=%.4f rad (%.2f deg) "
                  "→ target=%.4f rad (%.2f deg)",
                  current_goal_idx_ + 1, relative_yaws_.size(), start_yaw_,
                  start_yaw_ * 180.0 / M_PI, relative_yaws_[current_goal_idx_],
                  relative_yaws_[current_goal_idx_] * 180.0 / M_PI, target_yaw_,
                  target_yaw_ * 180.0 / M_PI);

      // Reset PID state
      integral_ = 0.0;
      last_error_ = 0.0;
    }

    double error = normalize_angle(target_yaw_ - current_yaw_);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "Turn %zu | yaw=%.4f rad | target=%.4f rad "
                         "| err=%.4f rad (%.2f deg)",
                         current_goal_idx_ + 1, current_yaw_, target_yaw_,
                         error, error * 180.0 / M_PI);

    if (std::fabs(error) < yaw_tolerance_) {
      stop_robot();
      RCLCPP_INFO(this->get_logger(), "Turn %zu reached. Pausing 1s...",
                  current_goal_idx_ + 1);
      rclcpp::sleep_for(1s);

      // Reanchor start_yaw_ to actual current yaw before next turn
      start_yaw_ = current_yaw_;
      current_goal_idx_++;
      goal_active_ = false;
      integral_ = 0.0;
      last_error_ = 0.0;

      if (current_goal_idx_ >= relative_yaws_.size()) {
        if (returning_home_) {
          stop_robot();
          RCLCPP_INFO(this->get_logger(), "All turns complete. Done!");
          timer_->cancel();
          rclcpp::shutdown();
        }
        // back to home as final turn
        returning_home_ = true;
        double return_delta = normalize_angle(home_yaw_ - current_yaw_);
        relative_yaws_.push_back(return_delta);
        RCLCPP_INFO(this->get_logger(),
                    "Returning to home yaw=%.4f rad (%.2f deg) | "
                    "delta=%.4f rad (%.2f deg)",
                    home_yaw_, home_yaw_ * 180.0 / M_PI, return_delta,
                    return_delta * 180.0 / M_PI);
      }
      return;
    }

    // PID
    integral_ += error;
    double derivative = error - last_error_;
    double control = kP_ * error + kI_ * integral_ + kD_ * derivative;

    // Clamp to max angular velocity
    control = std::clamp(control, -max_ang_vel_, max_ang_vel_);

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = control;
    twist_pub_->publish(cmd);

    last_error_ = error;
  }

  void stop_robot() {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 0.0;
    twist_pub_->publish(cmd);
  }

  double normalize_angle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  std::string prnt_yaws() {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < relative_yaws_.size(); i++) {
      oss << relative_yaws_[i] << " rad (" << relative_yaws_[i] * 180.0 / M_PI
          << " deg)";
      if (i != relative_yaws_.size() - 1)
        oss << ", ";
    }
    oss << "]";
    return oss.str();
  }

  int scene_number_;
  std::string odom_topic_;

  // PID
  double kP_, kI_, kD_;
  double yaw_tolerance_;
  double max_ang_vel_;

  // Odom
  double current_yaw_ = 0.0;
  bool odom_ready_ = false;
  bool returning_home_ = false;

  // Waypoints: relative yaw deltas in radians
  std::vector<double> relative_yaws_;
  size_t current_goal_idx_ = 0;
  bool goal_active_ = false;

  // Current target and anchoring
  double home_yaw_ = 0.0;
  double target_yaw_ = 0.0;
  double start_yaw_ = 0.0; // yaw at the moment current turn was activated

  // PID state
  double integral_ = 0.0;
  double last_error_ = 0.0;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  int scene_number = 1;
  if (argc > 1)
    scene_number = std::atoi(argv[1]);

  rclcpp::spin(std::make_shared<TurnController>(scene_number));
  rclcpp::shutdown();
  return 0;
}