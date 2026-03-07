#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

class DistanceController : public rclcpp::Node {

public:
  DistanceController(int scene_number)
      : Node("distance_controller_node"), scene_number_(scene_number) {

    odom_topic_ = this->declare_parameter<std::string>("odom_topic",
                                                       "/odometry/filtered");

    if (scene_number_ == 1) {
      kP_ = this->declare_parameter<float>("kP", 1.2f);
      kI_ = this->declare_parameter<float>("kI", 0.001f);
      kD_ = this->declare_parameter<float>("kD", 0.5f);
      goal_tolerance_ = this->declare_parameter<float>("goal_tol", 0.02f);
      max_speed_ = this->declare_parameter<float>("max_speed", 0.8f);
      max_accel_ = this->declare_parameter<float>("max_accel", 0.8f);
    } else {
      kP_ = this->declare_parameter<float>("kP", 1.2f);
      kI_ = this->declare_parameter<float>("kI", 0.001f);
      kD_ = this->declare_parameter<float>("kD", 0.05f);
      goal_tolerance_ = this->declare_parameter<float>("goal_tol", 0.05f);
      max_speed_ = this->declare_parameter<float>("max_speed", 0.2f);
      max_accel_ = this->declare_parameter<float>("max_accel", 0.2f);
    }

    SelectWaypoints();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          current_x_ = msg->pose.pose.position.x;
          current_y_ = msg->pose.pose.position.y;
          if (!odom_ready_) {
            RCLCPP_INFO(this->get_logger(),
                        "Odom ready — live home: (%.4f, %.4f)", current_x_,
                        current_y_);
            odom_ready_ = true;
          }
        });

    twist_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    // Control loop at 10 Hz
    timer_ = this->create_wall_timer(
        100ms, std::bind(&DistanceController::control_loop, this));

    last_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(),
                "PID: kP=%.3f kI=%.3f kD=%.3f | "
                "max_speed=%.2f m/s | max_accel=%.2f m/s²",
                kP_, kI_, kD_, max_speed_, max_accel_);
    RCLCPP_INFO(this->get_logger(), "Distance controller ready.");
  }

private:
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

      // Skip malformed entries
      if (!wp["x"] || !wp["y"]) {
        RCLCPP_WARN(this->get_logger(),
                    "Waypoint %zu missing x or y — skipping.", i);
        continue;
      }

      float x = wp["x"].as<float>();
      float y = wp["y"].as<float>();
      relative_waypoints_.push_back({x, y});

      RCLCPP_INFO(this->get_logger(), "  WP[%zu]: dx=%.4f  dy=%.4f", i, x, y);
    }

    RCLCPP_INFO(this->get_logger(), "%zu waypoints loaded.",
                relative_waypoints_.size());
  }

  void SelectWaypoints() {
    switch (scene_number_) {
    case 1: // Simulation
      file_path = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/resources/"
                  "waypoints/sim_waypoints_d.yaml";
      break;
    case 2: // Real robot
      file_path = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/resources/"
                  "waypoints/real_waypoints_d.yaml";
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid scene number: %d",
                   scene_number_);
      return;
    }

    LoadFromYaml(file_path);

    if (relative_waypoints_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No waypoints loaded — aborting.");
      return;
    }

    // Build return path
    int fwd_size = (int)relative_waypoints_.size();
    for (int i = fwd_size - 1; i >= 0; i--) {
      relative_waypoints_.push_back(
          {-relative_waypoints_[i].first, -relative_waypoints_[i].second});
    }

    RCLCPP_INFO(this->get_logger(),
                "Full path (%zu steps, forward + return): %s",
                relative_waypoints_.size(), prnt_waypoints().c_str());
  }

  // control_loop
  void control_loop() {
    if (!odom_ready_)
      return;
    if (current_goal_idx_ >= relative_waypoints_.size())
      return;

    // dt
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - last_time_).count();
    if (dt <= 0.0f || dt > 1.0f)
      dt = 0.1f;
    last_time_ = now;

    if (!goal_active_) {
      auto [rdx, rdy] = relative_waypoints_[current_goal_idx_];
      target_x_ = current_x_ + rdx;
      target_y_ = current_y_ + rdy;
      goal_active_ = true;

      RCLCPP_INFO(this->get_logger(),
                  "Goal %zu/%zu activated | "
                  "from (%.4f, %.4f) + delta (%.4f, %.4f) "
                  "→ target (%.4f, %.4f)",
                  current_goal_idx_ + 1, relative_waypoints_.size(), current_x_,
                  current_y_, rdx, rdy, target_x_, target_y_);

      // Reset PID state for new goal
      integral_x_ = 0.0f;
      integral_y_ = 0.0f;
      prev_err_x_ = 0.0f;
      prev_err_y_ = 0.0f;
      last_cmd_vx_ = 0.0f;
      last_cmd_vy_ = 0.0f;
    }

    // Compute errors
    float err_x = target_x_ - current_x_;
    float err_y = target_y_ - current_y_;
    float dist = std::sqrt(err_x * err_x + err_y * err_y);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "Goal %zu | err=(%.4f, %.4f) | dist=%.4f m",
                         current_goal_idx_ + 1, err_x, err_y, dist);

    // Goal reached
    if (dist < goal_tolerance_) {
      stop_robot();
      RCLCPP_INFO(this->get_logger(), "Goal %zu reached. Pausing 1s...",
                  current_goal_idx_ + 1);
      rclcpp::sleep_for(1s);

      current_goal_idx_++;
      goal_active_ = false;

      if (current_goal_idx_ >= relative_waypoints_.size()) {
        stop_robot();
        RCLCPP_INFO(this->get_logger(), "All waypoints complete. Done!");
        timer_->cancel();
        rclcpp::shutdown();
      }
      return;
    }

    // PID
    integral_x_ += err_x * dt;
    integral_y_ += err_y * dt;

    float deriv_x = (err_x - prev_err_x_) / dt;
    float deriv_y = (err_y - prev_err_y_) / dt;

    float target_vx = kP_ * err_x + kI_ * integral_x_ + kD_ * deriv_x;
    float target_vy = kP_ * err_y + kI_ * integral_y_ + kD_ * deriv_y;

    // Clamp to max speed
    target_vx = std::clamp(target_vx, -max_speed_, max_speed_);
    target_vy = std::clamp(target_vy, -max_speed_, max_speed_);

    // Acceleration limiting (slew rate) — prevents sudden lurches
    float cmd_vx = slew(last_cmd_vx_, target_vx, max_accel_, dt);
    float cmd_vy = slew(last_cmd_vy_, target_vy, max_accel_, dt);

    last_cmd_vx_ = cmd_vx;
    last_cmd_vy_ = cmd_vy;
    prev_err_x_ = err_x;
    prev_err_y_ = err_y;

    // Publish Velocity
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = cmd_vx;
    cmd.linear.y = cmd_vy;
    twist_pub_->publish(cmd);
  }

  // Slew-rate limiter: limits how fast velocity can change per step
  float slew(float current, float target, float max_rate, float dt) {
    float dv = target - current;
    float max_step = max_rate * dt;
    if (dv > max_step)
      return current + max_step;
    if (dv < -max_step)
      return current - max_step;
    return target;
  }

  void stop_robot() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0f;
    cmd.linear.y = 0.0f;
    twist_pub_->publish(cmd);
  }

  std::string prnt_waypoints() {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < relative_waypoints_.size(); i++) {
      oss << "(" << relative_waypoints_[i].first << ", "
          << relative_waypoints_[i].second << ")";
      if (i != relative_waypoints_.size() - 1)
        oss << ", ";
    }
    oss << "]";
    return oss.str();
  }

  int scene_number_;
  std::string odom_topic_;
  std::string file_path;

  // PID gains
  float kP_, kI_, kD_;
  float goal_tolerance_;
  float max_speed_;
  float max_accel_;

  // Odometry
  float current_x_ = 0.0f;
  float current_y_ = 0.0f;
  bool odom_ready_ = false;

  // Waypoints: stored as {dx, dy} relative displacements
  // includes both forward and return path
  std::vector<std::pair<float, float>> relative_waypoints_;
  size_t current_goal_idx_ = 0;
  bool goal_active_ = false;

  // Current absolute target
  float target_x_ = 0.0f;
  float target_y_ = 0.0f;

  // PID state
  float integral_x_ = 0.0f, integral_y_ = 0.0f;
  float prev_err_x_ = 0.0f, prev_err_y_ = 0.0f;

  // Velocity slew limiting
  float last_cmd_vx_ = 0.0f;
  float last_cmd_vy_ = 0.0f;

  // Timing
  std::chrono::steady_clock::time_point last_time_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  int scene_number = 1;
  if (argc > 1)
    scene_number = std::atoi(argv[1]);

  rclcpp::spin(std::make_shared<DistanceController>(scene_number));
  rclcpp::shutdown();
  return 0;
}