#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <filesystem>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <set>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tuple>
#include <vector>
#include <yaml-cpp/yaml.h>

class PIDMazeSolver : public rclcpp::Node {
public:
  PIDMazeSolver(int scene_number)
      : Node("pid_maze_solver"), scene_number_(scene_number) {

    relative_goals = readWaypointsYAML(scene_number_);

    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&PIDMazeSolver::odomCallback, this, std::placeholders::_1));

    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&PIDMazeSolver::scanCallback, this, std::placeholders::_1));

    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&PIDMazeSolver::controlLoop, this));
  }

private:
  struct Goal {
    float x;
    float y;
    float theta;
  };

  enum class Phase { TURNING, MOVING, CORRECTING };
  Phase phase = Phase::TURNING;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Goal> relative_goals;
  Goal current_target;

  int scene_number_;
  std::size_t current_goal_index = 0;
  bool initialized = false;
  bool goal_active = false;

  double current_x = 0.0, current_y = 0.0, current_yaw = 0.0;
  double start_x = 0.0, start_y = 0.0, start_yaw = 0.0;

  double prev_error_x = 0.0, integral_x = 0.0;
  double prev_error_y = 0.0, integral_y = 0.0;
  double prev_yaw_error = 0.0, integral_yaw = 0.0;

  float kp = 1.2, ki = 0.001, kd = 0.001;
  float kp_turn = 1.2, ki_turn = 0.001, kd_turn = 0.001;
  double max_linear_speed = 0.2;
  float goal_tolerance = 0.02;
  float yaw_tolerance = 0.052;

  float front_range_ = std::numeric_limits<float>::infinity();
  float left_range_ = std::numeric_limits<float>::infinity();
  float right_range_ = std::numeric_limits<float>::infinity();
  float back_range_ = std::numeric_limits<float>::infinity();
  float left_up_avg_ = std::numeric_limits<float>::quiet_NaN();
  float left_down_avg_ = std::numeric_limits<float>::quiet_NaN();
  float right_up_avg_ = std::numeric_limits<float>::quiet_NaN();
  float right_down_avg_ = std::numeric_limits<float>::quiet_NaN();
  int correction_counter_ = 0;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw);

    if (!initialized) {
      RCLCPP_INFO(this->get_logger(), "Initial pose: (%.3f, %.3f), yaw: %.3f",
                  current_x, current_y, current_yaw);
      initialized = true;
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const auto &ranges = msg->ranges;

    int idx_front = 0;
    int idx_left = 179;
    int idx_back = 359;
    int idx_right = 539;
    int idx_front_end = 719;

    front_range_ = (ranges[idx_front] + ranges[idx_front_end]) / 2.0;
    left_range_ = ranges[idx_left];
    back_range_ = ranges[idx_back];
    right_range_ = ranges[idx_right];

    auto calc_avg = [&](int start, int end) {
      float sum = 0.0f;
      int count = 0;
      for (int i = start; i <= end && i < static_cast<int>(ranges.size());
           ++i) {
        if (std::isfinite(ranges[i])) {
          sum += ranges[i];
          count++;
        }
      }
      return count > 0 ? sum / count : std::numeric_limits<float>::quiet_NaN();
    };

    left_down_avg_ = calc_avg(179, 224);
    left_up_avg_ = calc_avg(134, 179);
    right_down_avg_ = calc_avg(484, 539);
    right_up_avg_ = calc_avg(539, 584);
  }

  std::vector<Goal> readWaypointsYAML(int scene_number) {
    std::vector<Goal> waypoints;

    std::string package_share_directory =
        ament_index_cpp::get_package_share_directory("pid_maze_solver");
    std::string waypoint_file_name;

    switch (scene_number) {
    case 1:
      waypoint_file_name = "waypoints_sim.yaml";
      break;
    case 2:
      waypoint_file_name = "waypoints_real.yaml";
      break;
    case 3:
      waypoint_file_name = "reverse_waypoints_sim.yaml";
      break;
    case 4:
      waypoint_file_name = "reverse_waypoints_real.yaml";
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid scene number: %d",
                   scene_number);
      return waypoints;
    }

    std::string yaml_path =
        package_share_directory + "/waypoints/" + waypoint_file_name;
    RCLCPP_INFO(this->get_logger(), "Loading waypoints from: %s",
                yaml_path.c_str());

    try {
      YAML::Node config = YAML::LoadFile(yaml_path);
      if (config["waypoints"]) {
        for (const auto &wp : config["waypoints"]) {
          waypoints.push_back(
              {wp[0].as<float>(), wp[1].as<float>(), wp[2].as<float>()});
        }
      }
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s",
                   e.what());
    }

    return waypoints;
  }

  double normalizeAngle(double angle) {
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;
    return angle;
  }

  std::tuple<double, double, double> velocity2twist(double dphi, double dx,
                                                    double dy) {
    double cos_phi = std::cos(current_yaw);
    double sin_phi = std::sin(current_yaw);
    double wz = dphi;
    double vx = cos_phi * dx + sin_phi * dy;
    double vy = -sin_phi * dx + cos_phi * dy;
    return {wz, vx, vy};
  }

  void controlLoop() {
    if (!initialized)
      return;

    if (current_goal_index >= relative_goals.size()) {
      RCLCPP_INFO(this->get_logger(),
                  "All waypoints completed. Shutting down.");
      geometry_msgs::msg::Twist stop;
      vel_pub->publish(stop);
      timer_->cancel();
      rclcpp::shutdown();
      return;
    }
    const auto &goal = relative_goals[current_goal_index];

    if (!goal_active) {
      start_yaw = current_yaw;
      start_x = current_x;
      start_y = current_y;
      current_target = goal;
      goal_active = true;
      phase = Phase::TURNING;
      RCLCPP_INFO(this->get_logger(),
                  "New goal #%zu: turn %.3f rad, then move Δx=%.3f, Δy=%.3f",
                  current_goal_index, goal.theta, goal.x, goal.y);
    }

    if (phase == Phase::TURNING) {
      double target_yaw = normalizeAngle(start_yaw + goal.theta);
      double yaw_error = normalizeAngle(target_yaw - current_yaw);

      integral_yaw += yaw_error;
      double derivative = (yaw_error - prev_yaw_error) / 0.1;
      double angular_z =
          kp_turn * yaw_error + ki_turn * integral_yaw + kd_turn * derivative;
      angular_z = std::clamp(angular_z, -1.0, 1.0);

      geometry_msgs::msg::Twist twist;
      if (std::fabs(yaw_error) > yaw_tolerance) {
        twist.angular.z = angular_z;
        vel_pub->publish(twist);
        prev_yaw_error = yaw_error;
        return;
      }

      integral_yaw = 0.0;
      prev_yaw_error = 0.0;
      phase = Phase::MOVING;
      return;
    }

    if (phase == Phase::MOVING) {
      double target_x = start_x + goal.x;
      double target_y = start_y + goal.y;

      double error_x = target_x - current_x;
      double error_y = target_y - current_y;
      double distance = std::sqrt(error_x * error_x + error_y * error_y);

      integral_x += error_x;
      integral_y += error_y;

      double control_x =
          kp * error_x + ki * integral_x + kd * (error_x - prev_error_x) / 0.1;
      double control_y =
          kp * error_y + ki * integral_y + kd * (error_y - prev_error_y) / 0.1;

      control_x = std::clamp(control_x, -max_linear_speed, max_linear_speed);
      control_y = std::clamp(control_y, -max_linear_speed, max_linear_speed);

      auto [wz, vx, vy] = velocity2twist(0.0, control_x, control_y);

      geometry_msgs::msg::Twist vel;
      vel.linear.x = vx;
      vel.linear.y = vy;
      vel.angular.z = wz;

      if (left_range_ < 0.2) {
        vel.linear.y -= 0.05;
        RCLCPP_INFO(this->get_logger(), "Left too close");
      }
      if (right_range_ < 0.2) {
        vel.linear.y += 0.05;
        RCLCPP_INFO(this->get_logger(), "Right too close");
      }

      // Early stop if front obstacle is detected
      // Early stop if front obstacle is detected (except for waypoint 3)
      std::set<std::size_t> skip_early_stop = {3, 5, 9, 11, 12};
      float front_threshold = 0.22;
      if (current_goal_index == 13) {
        front_threshold = 0.18; // Special case for waypoint 13
      }

      if (skip_early_stop.count(current_goal_index) == 0 &&
          front_range_ < front_threshold) {
        RCLCPP_WARN(this->get_logger(),
                    "Front obstacle detected < %.2fm. Stopping early.",
                    front_threshold);
        geometry_msgs::msg::Twist stop;
        vel_pub->publish(stop);
        rclcpp::sleep_for(std::chrono::milliseconds(300));
        if (scene_number_ == 2 || scene_number_ == 4) {
          phase = Phase::CORRECTING;
        } else {
          current_goal_index++;
          goal_active = false;
          integral_x = 0.0;
          integral_y = 0.0;
          prev_error_x = 0.0;
          prev_error_y = 0.0;
          if (current_goal_index >= relative_goals.size()) {
            RCLCPP_INFO(this->get_logger(),
                        "All waypoints completed. Shutting down.");
            geometry_msgs::msg::Twist stop;
            vel_pub->publish(stop);
            timer_->cancel();
            rclcpp::shutdown();
            return;
          }
          phase = Phase::TURNING;
        }
        correction_counter_ = 0;
        return;
      }

      vel_pub->publish(vel);

      if (distance < goal_tolerance) {
        geometry_msgs::msg::Twist stop;
        for (int i = 0; i < 3; ++i) {
          vel_pub->publish(stop);
          rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
        if (scene_number_ == 2 || scene_number_ == 4) {
          phase = Phase::CORRECTING;
        } else {
          current_goal_index++;
          goal_active = false;
          integral_x = 0.0;
          integral_y = 0.0;
          prev_error_x = 0.0;
          prev_error_y = 0.0;
          phase = Phase::TURNING;
        }
        correction_counter_ = 0;
        return;
      }

      prev_error_x = error_x;
      prev_error_y = error_y;
      return;
    }

    if (phase == Phase::CORRECTING) {
      geometry_msgs::msg::Twist correction_vel;
      bool need_correction = false;

      if (front_range_ < 0.16) {
        correction_vel.linear.x -= 0.03;
        need_correction = true;
      }
      if (left_range_ < 0.2) {
        correction_vel.linear.y -= 0.03;
        need_correction = true;
      }
      if (right_range_ < 0.2) {
        correction_vel.linear.y += 0.03;
        need_correction = true;
      }

      // === Wall tilt correction based on closer side only ===
      float tilt_threshold = 0.015;
      float gain = 1.0;

      // Use only the closer wall for tilt correction
      if (left_range_ < right_range_) {
        if (std::isfinite(left_up_avg_) && std::isfinite(left_down_avg_)) {
          float diff = left_up_avg_ - left_down_avg_;
          if (std::fabs(diff) > tilt_threshold) {
            float tilt_correction = std::clamp(-gain * diff, -0.1f, 0.1f);
            correction_vel.angular.z += -tilt_correction;
            need_correction = true;

            RCLCPP_INFO(this->get_logger(),
                        "Left wall tilt correction (closer): diff=%.3f → "
                        "angular.z += %.3f",
                        diff, tilt_correction);
          }
        }
      } else if (right_range_ < left_range_) {
        if (std::isfinite(right_up_avg_) && std::isfinite(right_down_avg_)) {
          float diff = right_up_avg_ - right_down_avg_;
          if (std::fabs(diff) > tilt_threshold) {
            float tilt_correction = std::clamp(-gain * diff, -0.1f, 0.1f);
            correction_vel.angular.z += tilt_correction;
            need_correction = true;

            RCLCPP_INFO(this->get_logger(),
                        "Right wall tilt correction (closer): diff=%.3f → "
                        "angular.z += %.3f",
                        diff, tilt_correction);
          }
        }
      }

      if (need_correction) {
        vel_pub->publish(correction_vel);
        correction_counter_++;
        if (correction_counter_ > 2000) {
          RCLCPP_WARN(this->get_logger(), "Correction timeout. Moving on.");
          phase = Phase::TURNING;
          goal_active = false;
          current_goal_index++;
        }
        return;
      }

      geometry_msgs::msg::Twist stop;
      vel_pub->publish(stop);
      rclcpp::sleep_for(std::chrono::milliseconds(300));

      current_goal_index++;
      goal_active = false;
      integral_x = 0.0;
      integral_y = 0.0;
      prev_error_x = 0.0;
      prev_error_y = 0.0;

      if (current_goal_index >= relative_goals.size()) {
        vel_pub->publish(stop);
        timer_->cancel();
        rclcpp::shutdown();
      } else {
        phase = Phase::TURNING;
      }
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  if (argc < 2) {
    RCLCPP_ERROR(
        rclcpp::get_logger("pid_maze_solver"),
        "Usage: ros2 run pid_maze_solver pid_maze_solver <scene_number>");
    return 1;
  }
  int scene_number = std::atoi(argv[1]);
  rclcpp::spin(std::make_shared<PIDMazeSolver>(scene_number));
  rclcpp::shutdown();
  return 0;
}
