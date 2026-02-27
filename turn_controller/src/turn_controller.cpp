#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <functional>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class TurnController : public rclcpp::Node {

public:
  TurnController(int scene_number)
      : Node("turn_controller_node"), scene_number_(scene_number) {

    num_waypoints_ = this->declare_parameter<int>("num_waypoints", -1);
    odom_topic_ = this->declare_parameter<std::string>("odom_topic",
                                                       "/odometry/filtered");

    // Waypoint sequence:
    auto seq = this->declare_parameter<std::vector<long int>>(
        "waypoint_sequence", std::vector<long int>{13, 6, 3});
    for (auto v : seq) {
      waypoint_sequence_.push_back((int)v);
    }

    RCLCPP_INFO(this->get_logger(), "Waypoint Sequences array: %s",
                prnt_vector(waypoint_sequence_).c_str());

    // PID gains
    if (scene_number_ == 1) {
      // slow and steady : kP:=2.0 -p kI:=0.001 -p kD:=1.1
      // Fast and stable yet : kP:=3.4 -p kI:=0.0001 -p kD:=2.2
      kP_ = this->declare_parameter<float>("kP", 3.0f);
      kI_ = this->declare_parameter<float>("kI", 0.0001f);
      kD_ = this->declare_parameter<float>("kD", 2.0f);
      max_ang_vel_ = 1.5f;
    } else {
      kP_ = this->declare_parameter<float>("kP", 0.8f);
      kI_ = this->declare_parameter<float>("kI", 0.005f);
      kD_ = this->declare_parameter<float>("kD", 0.5f);
      max_ang_vel_ = 0.5f;
    }

    LoadWaypointsFromJson();

    timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = this->create_wall_timer(
        1s, std::bind(&TurnController::timer_callback, this),
        timer_callback_group_);

    odom_sub_options_.callback_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          // Store position and yaw (from quaternion)
          current_pose_(0) = msg->pose.pose.position.x;
          current_pose_(1) = msg->pose.pose.position.y;
          float qz = msg->pose.pose.orientation.z;
          float qw = msg->pose.pose.orientation.w;
          current_pose_(2) = 2.0f * std::atan2(qz, qw); // yaw
          initial_odom_received_ = true;
        },
        odom_sub_options_);

    twist_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    RCLCPP_INFO(this->get_logger(),
                "PID gains: kP=%.3f kI=%.3f kD=%.3f | max_ang_vel=%.3f", kP_,
                kI_, kD_, max_ang_vel_);
    RCLCPP_INFO(this->get_logger(), "TurnController initialization complete.");
  }

private:
  void LoadWaypointsFromJson() {
    // Select file based on scene
    std::string file_path;
    switch (scene_number_) {
    case 1:
      file_path = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/resources/"
                  "waypoints/sim_waypoints_0.2.json";
      break;
    case 2:
      file_path = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/resources/"
                  "waypoints/real_waypoints_0.2.json";
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid scene number: %d",
                   scene_number_);
      return;
    }

    // Open file
    std::ifstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open: %s", file_path.c_str());
      return;
    }

    nlohmann::json json_data;
    file >> json_data;

    // Parsing all waypoints
    int total = (int)json_data.size();
    for (int i = 0; i < total; i++) {
      std::string key = std::to_string(i);
      for (auto &entry : json_data) {
        if (entry.contains(key)) {
          auto &wp = entry[key]["data"][odom_topic_];
          float x = wp["position"]["x"];
          float y = wp["position"]["y"];
          float qz = wp["orientation"]["z"];
          float qw = wp["orientation"]["w"];
          float yaw = 2.0f * std::atan2(qz, qw); // convert quaternion to yaw

          all_waypoints_.push_back(Eigen::Vector3f(x, y, yaw));
          break;
        }
      }
    }
    RCLCPP_INFO(this->get_logger(), "All Waypoints: %s",
                prnt_v_eigen(all_waypoints_).c_str());

    // Pre-compute yaw deltas between consecutive waypoints (for relative mode)
    for (size_t i = 0; i + 1 < all_waypoints_.size(); i++) {
      float delta_yaw = all_waypoints_[i + 1](2) - all_waypoints_[i](2);
      // Normalize delta to -PI to PI
      while (delta_yaw > M_PI)
        delta_yaw -= 2.0f * M_PI;
      while (delta_yaw < -M_PI)
        delta_yaw += 2.0f * M_PI;
      relative_yaw_deltas_.push_back(delta_yaw);
    }
    RCLCPP_INFO(this->get_logger(), "Relative yaw deltas: %s",
                prnt_vector(relative_yaw_deltas_).c_str());
    RCLCPP_INFO(this->get_logger(),
                "Loaded %zu waypoints | %zu yaw deltas pre-computed",
                all_waypoints_.size(), relative_yaw_deltas_.size());
  }

  // Check for Drift and build execution yaw list
  // Absolute mode: use recorded yaws directly
  // Relative mode: shift all yaws by (live_yaw - json_home_yaw)
  void BuildExecutionYaws() {
    float json_home_yaw = all_waypoints_[0](2);
    float live_yaw = current_pose_(2);
    float yaw_offset = live_yaw - json_home_yaw;

    // Normalize yaw offset to [-π, π]
    while (yaw_offset > M_PI)
      yaw_offset -= 2.0f * M_PI;
    while (yaw_offset < -M_PI)
      yaw_offset += 2.0f * M_PI;

    // Check position drift (same as distance controller)
    Eigen::Vector2f json_home_pos(all_waypoints_[0](0), all_waypoints_[0](1));
    Eigen::Vector2f live_pos(current_pose_(0), current_pose_(1));
    float drift = (json_home_pos - live_pos).norm();

    RCLCPP_INFO(this->get_logger(),
                "Face toward waypoint position | drift=%.4f m", drift);

    relative_mode_ = (drift > ODOM_DRIFT_THRESHOLD);
    RCLCPP_INFO(this->get_logger(), "%s mode",
                relative_mode_ ? "RELATIVE" : "ABSOLUTE");

    // Position offset for relative mode(shift waypoint positions by drift)
    Eigen::Vector2f pos_offset(0.0f, 0.0f);
    if (relative_mode_) {
      // Offset = live_pos - json_home_pos
      // Applied to all waypoint positions so they align with live odom frame
      pos_offset = live_pos - json_home_pos;
      RCLCPP_WARN(this->get_logger(),
                  "Applying position offset (%.4f, %.4f) to waypoints",
                  pos_offset.x(), pos_offset.y());
    }

    // Build execution yaw list from waypoint_sequence_ with offset applied
    execution_yaws_.clear();
    for (int idx : waypoint_sequence_) {
      if (idx < 0 || idx >= (int)all_waypoints_.size()) {
        RCLCPP_WARN(this->get_logger(),
                    "Waypoint index %d out of range, skipping.", idx);
        continue;
      }

      // Waypoint position (compensated for odom drift in relative mode)
      float wp_x = all_waypoints_[idx](0) + pos_offset.x();
      float wp_y = all_waypoints_[idx](1) + pos_offset.y();

      // Angle from current position toward waypoint position
      float dx = wp_x - current_pose_(0);
      float dy = wp_y - current_pose_(1);
      float target_yaw = std::atan2(dy, dx); // world-frame angle to face

      execution_yaws_.push_back(target_yaw);

      RCLCPP_INFO(this->get_logger(), "Yaws to be Executed: %s",
                  prnt_vector(execution_yaws_).c_str());
      RCLCPP_INFO(this->get_logger(),
                  "WP[%d] recorded_yaw=%.4f + offset=%.4f → target=%.4f rad",
                  idx, all_waypoints_[idx](2), yaw_offset, target_yaw);
    }
  }

  static float NormalizeAngle(float angle) {
    while (angle > M_PI)
      angle -= 2.0f * M_PI;
    while (angle < -M_PI)
      angle += 2.0f * M_PI;
    return angle;
  }

  void StopRobot() {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 0.0f;
    cmd.linear.x = 0.0f;
    cmd.linear.y = 0.0f;
    twist_pub_->publish(cmd);
  }

  void timer_callback() {
    timer_->cancel();
    // Wait until first real odom reading arrives
    rclcpp::Rate wait_rate(10ms);
    while (!initial_odom_received_ && rclcpp::ok()) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                           "Waiting for odometry...");
      wait_rate.sleep();
    }

    RCLCPP_INFO(this->get_logger(),
                "Live odom ready: pos=(%.4f, %.4f) yaw=%.4f rad",
                current_pose_(0), current_pose_(1), current_pose_(2));

    // Build target yaw list with drift compensation
    BuildExecutionYaws();

    if (execution_yaws_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "No execution yaws — check waypoint_sequence.");
      return;
    }

    // Store initial yaw to return to at the end
    float initial_yaw = current_pose_(2);

    rclcpp::Rate rate(100ms); // 10 Hz control loop
    geometry_msgs::msg::Twist cmd_vel;

    float err_yaw;
    float prev_err_yaw = 0.0f;
    float sum_I = 0.0f;
    float dt;
    rclcpp::Time current_time, prev_time;

    for (size_t i = 0; i < execution_yaws_.size(); i++) {
      float target_yaw = execution_yaws_[i];

      RCLCPP_INFO(this->get_logger(),
                  "Turn %zu/%zu → target yaw=%.4f rad (%.1f deg)", i + 1,
                  execution_yaws_.size(), target_yaw,
                  target_yaw * 180.0f / M_PI);

      // Reset PID state for each turn
      prev_err_yaw = 0.0f;
      // prev_err_yaw = NormalizeAngle(target_yaw - current_pose_(2));
      sum_I = 0.0f;
      prev_time = this->get_clock()->now();

      // PID loop until within yaw tolerance
      err_yaw = NormalizeAngle(target_yaw - current_pose_(2));
      while (std::abs(err_yaw) >= YAW_TOLERANCE && rclcpp::ok()) {

        err_yaw = NormalizeAngle(target_yaw - current_pose_(2));
        current_time = this->get_clock()->now();
        dt = (current_time - prev_time).seconds();

        if (dt <= 0.0f) {
          rate.sleep();
          continue;
        }

        // Integral term
        sum_I += err_yaw * dt;
        sum_I = std::clamp(sum_I, -1.0f, 1.0f); // anti-windup

        // Derivative term (rate of yaw error change)
        float derivative = (err_yaw - prev_err_yaw) / dt;

        // PID output → angular velocity
        float angular_vel = kP_ * err_yaw + kI_ * sum_I + kD_ * derivative;

        // Clamp to max angular velocity
        angular_vel = std::clamp(angular_vel, -max_ang_vel_, max_ang_vel_);

        // Strictly in-place rotation — no translation
        cmd_vel.linear.x = 0.0f;
        cmd_vel.linear.y = 0.0f;
        cmd_vel.angular.z = angular_vel;
        twist_pub_->publish(cmd_vel);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "Turn %zu | err=%.4f rad (%.1f deg) | "
                             "P=%.3f I=%.3f D=%.3f | ang_vel=%.3f",
                             i + 1, err_yaw, err_yaw * 180.0f / M_PI,
                             kP_ * err_yaw, kI_ * sum_I, kD_ * derivative,
                             angular_vel);

        prev_err_yaw = err_yaw;
        prev_time = current_time;
        rate.sleep();
      }

      // Stop and pause at each target yaw
      StopRobot();
      RCLCPP_INFO(this->get_logger(),
                  "Turn %zu reached | final yaw=%.4f rad | err=%.4f rad", i + 1,
                  current_pose_(2), err_yaw);
      rclcpp::sleep_for(1s);
    }

    RCLCPP_INFO(this->get_logger(), "Returning to initial yaw=%.4f rad",
                initial_yaw);

    prev_err_yaw = 0.0f;
    sum_I = 0.0f;
    prev_time = this->get_clock()->now();

    err_yaw = NormalizeAngle(initial_yaw - current_pose_(2));
    while (std::abs(err_yaw) >= YAW_TOLERANCE && rclcpp::ok()) {

      err_yaw = NormalizeAngle(initial_yaw - current_pose_(2));
      current_time = this->get_clock()->now();
      dt = (current_time - prev_time).seconds();

      if (dt <= 0.0f) {
        rate.sleep();
        continue;
      }

      sum_I += err_yaw * dt;
      sum_I = std::clamp(sum_I, -1.0f, 1.0f);

      float derivative = (err_yaw - prev_err_yaw) / dt;
      float angular_vel = kP_ * err_yaw + kI_ * sum_I + kD_ * derivative;
      angular_vel = std::clamp(angular_vel, -max_ang_vel_, max_ang_vel_);

      cmd_vel.linear.x = 0.0f;
      cmd_vel.linear.y = 0.0f;
      cmd_vel.angular.z = angular_vel;
      twist_pub_->publish(cmd_vel);

      prev_err_yaw = err_yaw;
      prev_time = current_time;
      rate.sleep();
    }

    // Final stop
    StopRobot();
    RCLCPP_INFO(this->get_logger(),
                "Returned to initial yaw. All turns complete. Mission done!");
    rclcpp::shutdown();
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

  std::string prnt_v_eigen(const std::vector<Eigen::Vector3f> &vec) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
      oss << "(" << vec[i].x() << ", " << vec[i].y() << ", " << vec[i].z()
          << ")";
      if (i != vec.size() - 1)
        oss << ", ";
    }
    oss << "]";
    return oss.str();
  }

  static constexpr float ODOM_DRIFT_THRESHOLD = 0.05f; // metres
  static constexpr float YAW_TOLERANCE = 0.01f;        // radians

  float kP_;          // Proportional
  float kI_;          // Integral
  float kD_;          // Derivative
  float max_ang_vel_; // rad/s cap

  bool initial_odom_received_ = false;
  bool relative_mode_ = false;
  Eigen::Vector3f current_pose_; // (x, y, yaw)

  std::vector<Eigen::Vector3f> all_waypoints_; // (x, y, yaw) from JSON
  std::vector<float> relative_yaw_deltas_;     // yaw[i+1] - yaw[i]
  std::vector<int> waypoint_sequence_;         // indices to visit
  std::vector<float> execution_yaws_;          // final target yaws

  int scene_number_;
  int num_waypoints_;
  std::string odom_topic_;

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::SubscriptionOptions odom_sub_options_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  int scene_number = 1;
  if (argc > 1) {
    scene_number = std::atoi(argv[1]);
  }

  auto node = std::make_shared<TurnController>(scene_number);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}