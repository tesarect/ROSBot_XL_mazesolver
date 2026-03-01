#include "pid_maze_solver/pid_maze_solver.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "yaml-cpp/yaml.h"
#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <nlohmann/json.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

PIDMazeSolver::PIDMazeSolver(int scene_number,
                             const rclcpp::NodeOptions &options)
    : Node("maze_solver_node", options), scene_number_(scene_number) {

  LoadParameters();
  LoadWaypointsYaml();

  timer_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ = this->create_wall_timer(
      1s, std::bind(&PIDMazeSolver::timer_callback, this),
      timer_callback_group_);

  odom_sub_options_.callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  odom_sub_ = this->create_subscription<Odom>(
      odom_topic_, 10,
      [this](const Odom::SharedPtr msg) {
        // Store position and yaw (from quaternion)
        current_pose_.x = msg->pose.pose.position.x;
        current_pose_.y = msg->pose.pose.position.y;
        float qz = msg->pose.pose.orientation.z;
        float qw = msg->pose.pose.orientation.w;
        current_pose_.yaw = 2.0f * std::atan2(qz, qw); // yaw
        initial_odom_received_ = true;
      },
      odom_sub_options_);

  twist_pub_ = this->create_publisher<Twist>("cmd_vel", 1);

  laser_sub_options_.callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  laser_sub_ = this->create_subscription<Laser>(
      laser_topic_, 10,
      std::bind(&PIDMazeSolver::laserCallback, this, std::placeholders::_1),
      laser_sub_options_);

  RCLCPP_INFO(this->get_logger(),
              "PID gains: TkP=%.3f TkI=%.3f TkD=%.3f | max_ang_vel=%.3f", TkP_,
              TkI_, TkD_, max_ang_vel_);
  RCLCPP_INFO(this->get_logger(), "PIDMazeSolver initialization complete.");

  // std::exit(EXIT_FAILURE); // ⛔  s t o p
}

void PIDMazeSolver::initLaser(const Laser::SharedPtr msg) {
  angle_min_ = msg->angle_min;
  angle_increment_ = msg->angle_increment;
  num_rays_ = static_cast<int>(msg->ranges.size());

  auto toIdx = [&](double deg) {
    int idx = static_cast<int>(
        std::round((deg * M_PI / 180.0 - angle_min_) / angle_increment_));
    return std::clamp(idx, 0, num_rays_ - 1);
  };

  if (scene_number_ == 1) {
    north_idx_ = toIdx(180.0);
    south_idx_ = toIdx(0.0);
    east_idx_ = toIdx(90.0);
    west_idx_ = toIdx(-90.0);
    se_idx_ = toIdx(45.0);
    sw_idx_ = toIdx(-45.0);
    ne_idx_ = toIdx(135.0);
    nw_idx_ = toIdx(-135.0);
  } else {
    north_idx_ = toIdx(0.0);
    south_idx_ = toIdx(180.0);
    east_idx_ = toIdx(90.0);
    west_idx_ = toIdx(-90.0);
    ne_idx_ = toIdx(45.0);
    nw_idx_ = toIdx(-45.0);
    se_idx_ = toIdx(135.0);
    sw_idx_ = toIdx(-135.0);
  }

  front_idx_ = north_idx_;

  band_half_ =
      std::max(1, static_cast<int>(std::round((BAND_DEGREES * M_PI / 180.0) /
                                              angle_increment_)));

  //   RCLCPP_INFO(
  //       get_logger(),
  //       "[LaserInit] rays=%d  angle_min=%.2f°  inc=%.3f°  band=±%d rays
  //       (±%.1f°)", num_rays_, angle_min_ * 180.0 / M_PI, angle_increment_ *
  //       180.0 / M_PI, band_half_, BAND_DEGREES);

  //   RCLCPP_INFO(get_logger(),
  //               "  Indices → N:%d  NE:%d  E:%d  SE:%d  S:%d  SW:%d  W:%d
  //               NW:%d", north_idx_, ne_idx_, east_idx_, se_idx_, south_idx_,
  //               sw_idx_, west_idx_, nw_idx_);

  // Quick sanity: print band-averaged range for each direction on first scan
  // so you can immediately verify against known physical distances on the
  // robot.
  //   auto quickAvg = [&](int center) {
  //     double sum = 0.0;
  //     int cnt = 0;
  //     for (int i = std::max(0, center - band_half_);
  //          i <= std::min(num_rays_ - 1, center + band_half_); ++i) {
  //       float r = msg->ranges[i];
  //       if (std::isfinite(r) && r > 0.01f && r < msg->range_max)
  //         sum += r, ++cnt;
  //     }
  //     return cnt > 0 ? sum / cnt : -1.0;
  //   };

  //   RCLCPP_INFO(get_logger(),
  //               "  First-scan ranges (m) → N:%.2f  NE:%.2f  E:%.2f  SE:%.2f"
  //               "  S:%.2f  SW:%.2f  W:%.2f  NW:%.2f",
  //               quickAvg(north_idx_), quickAvg(ne_idx_), quickAvg(east_idx_),
  //               quickAvg(se_idx_), quickAvg(south_idx_), quickAvg(sw_idx_),
  //               quickAvg(west_idx_), quickAvg(nw_idx_));

  //   RCLCPP_INFO(
  //       get_logger(),
  //       "  Place robot facing a known wall (e.g. wall directly in front =
  //       N)." " N should show shortest range. Adjust toIdx args above if
  //       mismatch.");

  laser_initialized_ = true;
}

double PIDMazeSolver::bandAvg(const std::vector<float> &ranges, int center,
                              int half, double clip, float range_max) {
  double sum = 0.0;
  int cnt = 0;
  int n = static_cast<int>(ranges.size());
  for (int i = std::max(0, center - half); i <= std::min(n - 1, center + half);
       ++i) {
    float r = ranges[i];
    if (std::isfinite(r) && r > 0.01f && r < range_max)
      sum += std::min((double)r, clip), ++cnt;
  }
  return cnt > 0 ? sum / cnt : clip;
}

void PIDMazeSolver::laserCallback(const Laser::SharedPtr msg) {
  if (!laser_initialized_)
    initLaser(msg);

  const auto &ranges = msg->ranges;

  dN = bandAvg(ranges, north_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  dNE = bandAvg(ranges, ne_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  dE = bandAvg(ranges, east_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  dSE = bandAvg(ranges, se_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  dS = bandAvg(ranges, south_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  dSW = bandAvg(ranges, sw_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  dW = bandAvg(ranges, west_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  dNW = bandAvg(ranges, nw_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);

  //   RCLCPP_INFO(get_logger(),
  //               "Dist(m)  N:%.2f  NE:%.2f  E:%.2f  SE:%.2f  S:%.2f  SW:%.2f "
  //               "W:%.2f  NW:%.2f",
  //               dN, dNE, dE, dSE, dS, dSW, dW, dNW);

  //   if (dN < WALL_CLOSE_THRESH)
  //     RCLCPP_WARN(get_logger(), "WALL CLOSE → N  (%.2f m)", dN);
  //   if (dNE < WALL_CLOSE_THRESH)
  //     RCLCPP_WARN(get_logger(), "WALL CLOSE → NE (%.2f m)", dNE);
  //   if (dE < WALL_CLOSE_THRESH)
  //     RCLCPP_WARN(get_logger(), "WALL CLOSE → E  (%.2f m)", dE);
  //   if (dSE < WALL_CLOSE_THRESH)
  //     RCLCPP_WARN(get_logger(), "WALL CLOSE → SE (%.2f m)", dSE);
  //   if (dS < WALL_CLOSE_THRESH)
  //     RCLCPP_WARN(get_logger(), "WALL CLOSE → S  (%.2f m)", dS);
  //   if (dSW < WALL_CLOSE_THRESH)
  //     RCLCPP_WARN(get_logger(), "WALL CLOSE → SW (%.2f m)", dSW);
  //   if (dW < WALL_CLOSE_THRESH)
  //     RCLCPP_WARN(get_logger(), "WALL CLOSE → W  (%.2f m)", dW);
  //   if (dNW < WALL_CLOSE_THRESH)
  //     RCLCPP_WARN(get_logger(), "WALL CLOSE → NW (%.2f m)", dNW);

  // ── 3. Corridor balance (equidistant between opposite walls) ────────────
  //  Positive error → robot drifted toward the first direction listed
  double ew_drift = dE - dW; // >0 → too close to W, drift right
  double ne_nw_drift = dNE - dNW;
  double se_sw_drift = dSE - dSW;

  RCLCPP_DEBUG(get_logger(), "Drift  E-W=%.3f  NE-NW=%.3f  SE-SW=%.3f",
               ew_drift, ne_nw_drift, se_sw_drift);

  // ── 4. Detect continuous openings across the full scan ──────────────────
  openings_.clear();
  bool in_opening = false;
  int open_start = 0;
  double range_acc = 0.0;
  int range_cnt = 0;

  auto flushOpening = [&](int end_idx) {
    if (!in_opening || range_cnt == 0)
      return;
    double width_deg = (end_idx - open_start) * angle_increment_ * 180.0 / M_PI;
    if (width_deg >= OPENING_MIN_DEG) {
      double center_rad =
          angle_min_ + (open_start + end_idx) * 0.5 * angle_increment_;
      double center_deg = center_rad * 180.0 / M_PI;
      // wrap to (-180, 180]
      while (center_deg > 180.0)
        center_deg -= 360.0;
      while (center_deg <= -180.0)
        center_deg += 360.0;
      openings_.push_back({center_deg, width_deg, range_acc / range_cnt});
    }
    in_opening = false;
    range_acc = 0.0;
    range_cnt = 0;
  };

  for (int i = 0; i < num_rays_; ++i) {
    float r = ranges[i];
    bool open = std::isfinite(r) && r > static_cast<float>(OPENING_MIN_RANGE) &&
                r < msg->range_max;
    if (open) {
      if (!in_opening) {
        in_opening = true;
        open_start = i;
      }
      range_acc += r;
      ++range_cnt;
    } else {
      flushOpening(i);
    }
  }
  flushOpening(num_rays_ - 1);

  // ── 5. Log openings ─────────────────────────────────────────────────────
  //   for (auto &op : openings_)
  //     RCLCPP_INFO(get_logger(),
  //                 "Opening  center=%.1f°  width=%.1f°  avg_range=%.2f m",
  //                 op.center_angle_deg, op.width_deg, op.avg_range);

  // wall_dist values (dN … dNW) and openings_ are now ready for your
  // maze-solver / motion controller to consume this tick.
}

void PIDMazeSolver::LoadParameters() {
  num_waypoints_ = this->declare_parameter<int>("num_waypoints", -1);
  odom_topic_ =
      this->declare_parameter<std::string>("odom_topic", "/odometry/filtered");
  laser_topic_ = this->declare_parameter<std::string>("laser_topic", "/scan");
  //   laser_flip_ = this->declare_parameter<bool>("laser_flip", false);

  laser_offset_deg_ =
      this->declare_parameter<double>("laser_offset_deg", 180.0);

  // Turn PID gains
  TkP_ = this->declare_parameter<float>("TkP", 3.0f);
  TkI_ = this->declare_parameter<float>("TkI", 0.0001f);
  TkD_ = this->declare_parameter<float>("TkD", 2.0f);
  // Distance PID gains
  DkP_ = this->declare_parameter<float>("DkP", 0.8f);
  DkI_ = this->declare_parameter<float>("DkI", 0.005f);
  DkD_ = this->declare_parameter<float>("DkD", 0.5f);

  max_ang_vel_ = this->declare_parameter<float>("max_ang_vel", 0.5f);
  odom_drift_threshold_ =
      this->declare_parameter<float>("odom_drift_threshold", 0.05f);
  yaw_tolerance_ = this->declare_parameter<float>("yaw_tolerance", 0.05f);

  RCLCPP_INFO(this->get_logger(), " -Parameters Initialized");
}

void PIDMazeSolver::LoadWaypointsYaml() {
  // Select file based on scene
  std::string file_path;
  switch (scene_number_) {
  case 1:
    file_path = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/resources/"
                "waypoints/sim_waypoints.yaml";
    break;
  case 2:
    file_path = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/resources/"
                "waypoints/real_waypoints.yaml";
    break;
  case 3:
    file_path = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/resources/"
                "waypoints/real_waypoints.yaml";
    break;
  case 4:
    file_path = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/resources/"
                "waypoints/real_waypoints.yaml";
    break;
  default:
    RCLCPP_ERROR(this->get_logger(), "Invalid scene number: %d", scene_number_);
    return;
  }

  // Open file
  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open: %s", file_path.c_str());
    return;
  }

  YAML::Node yaml_data = YAML::LoadFile(file_path);

  // Parsing all waypoints
  int total = yaml_data.size();
  for (int i = 0; i < total; i++) {
    std::string key = std::to_string(i);
    for (const auto &entry : yaml_data) {
      if (entry[key]) {
        YAML::Node wp = entry[key]["data"][odom_topic_];
        float x = wp["position"]["x"].as<float>();
        float y = wp["position"]["y"].as<float>();
        float qz = wp["orientation"]["z"].as<float>();
        float qw = wp["orientation"]["w"].as<float>();
        float yaw = 2.0f * std::atan2(qz, qw);

        all_waypoints_.push_back(PoseOrient(x, y, yaw));
        break;
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), " -All Waypoint Loaded");
  RCLCPP_INFO(this->get_logger(), "All waypoints: %s",
              prnt_waypoints(all_waypoints_).c_str());
  //   RCLCPP_INFO(this->get_logger(), "All Waypoints: %s",
  //               prnt_v_eigen(all_waypoints_).c_str());

  // // Pre-compute yaw deltas between consecutive waypoints (for relative
  // mode) for (size_t i = 0; i + 1 < all_waypoints_.size(); i++) {
  //   float delta_yaw = all_waypoints_[i + 1](2) - all_waypoints_[i](2);
  //   // Normalize delta to -PI to PI
  //   while (delta_yaw > M_PI)
  //     delta_yaw -= 2.0f * M_PI;
  //   while (delta_yaw < -M_PI)
  //     delta_yaw += 2.0f * M_PI;
  //   relative_yaw_deltas_.push_back(delta_yaw);
  // }
  // RCLCPP_INFO(this->get_logger(), "Relative yaw deltas: %s",
  //             prnt_vector(relative_yaw_deltas_).c_str());
  // RCLCPP_INFO(this->get_logger(),
  //             "Loaded %zu waypoints | %zu yaw deltas pre-computed",
  //             all_waypoints_.size(), relative_yaw_deltas_.size());
  waypoint_selection();
}

void PIDMazeSolver::waypoint_selection() {
  fwd_waypoint_indices_seq_.clear();
  std::vector<size_t> ignore;
  if (scene_number_ == 2) {
    ignore = {2};
  }
  for (size_t idx = 0; idx < all_waypoints_.size(); ++idx) {
    // if ignore is empty, select everything
    if (ignore.empty() ||
        std::find(ignore.begin(), ignore.end(), idx) == ignore.end()) {
      fwd_waypoint_indices_seq_.push_back(idx);
    }
  }

  if (fwd_waypoint_indices_seq_.empty()) {
    RCLCPP_ERROR(
        get_logger(),
        "Unable to prepare waypoint for reverse and combined waypoints");
    return;
  }

  rev_waypoint_indices_seq_ = fwd_waypoint_indices_seq_;
  std::reverse(rev_waypoint_indices_seq_.begin(),
               rev_waypoint_indices_seq_.end());

  combined_waypoint_indices_seq_ = fwd_waypoint_indices_seq_;

  combined_waypoint_indices_seq_.insert(
      combined_waypoint_indices_seq_.end(),
      std::next(fwd_waypoint_indices_seq_.rbegin()),
      fwd_waypoint_indices_seq_.rend());

  printWaypointsSequence(fwd_waypoint_indices_seq_, "Forward");
  printWaypointsSequence(rev_waypoint_indices_seq_, "Reverse");
  printWaypointsSequence(combined_waypoint_indices_seq_, "Combined");
  std::exit(EXIT_FAILURE); // ⛔  s t o p

  //   Usage: to access ith waypoint in
  //   const PoseOrient& target =
  //     all_waypoints_[fwd_waypoint_indices_seq_[current_seq_index_]];
}

PIDMazeSolver::LaserDirections
PIDMazeSolver::compute_laser_indices(const Laser &scan) {
  int n = scan.ranges.size();
  LaserDirections dirs;

  // Compute the angle for each index
  auto angle_of_index = [&](int i) {
    return scan.angle_min + i * scan.angle_increment;
  };

  // Find index closest to desired robot direction (X forward, X backward, Y
  // left, Y right)
  auto find_closest_index = [&](float target_angle) {
    int idx = 0;
    float min_diff = std::numeric_limits<float>::max();
    for (int i = 0; i < n; i++) {
      float diff = std::abs(NormalizeAngle(angle_of_index(i) - target_angle));
      if (diff < min_diff) {
        min_diff = diff;
        idx = i;
      }
    }
    return idx;
  };

  dirs.front_index = find_closest_index(0.0f);    // +X forward
  dirs.back_index = find_closest_index(M_PI);     // ±180° backward
  dirs.left_index = find_closest_index(M_PI_2);   // +Y left
  dirs.right_index = find_closest_index(-M_PI_2); // -Y right

  RCLCPP_INFO(this->get_logger(),
              " ---------------------------Laser Initialized");
  return dirs;
}

// Check for Drift and build execution yaw list
// Absolute mode: use recorded yaws directly
// Relative mode: shift all yaws by (live_yaw - json_home_yaw)
void PIDMazeSolver::BuildExecutionYaws() {

  float json_home_yaw = all_waypoints_[0].yaw;
  float live_yaw = current_pose_.yaw;

  float yaw_offset = live_yaw - json_home_yaw;

  while (yaw_offset > M_PI)
    yaw_offset -= 2.0f * M_PI;
  while (yaw_offset < -M_PI)
    yaw_offset += 2.0f * M_PI;

  // Compute position drift manually (no Eigen)
  float dx_home = current_pose_.x - all_waypoints_[0].x;
  float dy_home = current_pose_.y - all_waypoints_[0].y;

  float drift = std::sqrt(dx_home * dx_home + dy_home * dy_home);

  RCLCPP_INFO(this->get_logger(),
              "Face toward waypoint position | drift=%.4f m", drift);

  relative_mode_ = (drift > odom_drift_threshold_);

  RCLCPP_INFO(this->get_logger(), "%s mode",
              relative_mode_ ? "RELATIVE" : "ABSOLUTE");

  float offset_x = 0.0f;
  float offset_y = 0.0f;

  if (relative_mode_) {
    offset_x = dx_home;
    offset_y = dy_home;

    RCLCPP_WARN(this->get_logger(),
                "Applying position offset (%.4f, %.4f) to waypoints", offset_x,
                offset_y);
  }

  execution_yaws_.clear();

  for (int idx : waypoint_sequence_) {
    if (idx < 0 || idx >= (int)all_waypoints_.size()) {
      RCLCPP_WARN(this->get_logger(),
                  "Waypoint index %d out of range, skipping.", idx);
      continue;
    }

    float wp_x = all_waypoints_[idx].x + offset_x;
    float wp_y = all_waypoints_[idx].y + offset_y;

    float dx = wp_x - current_pose_.x;
    float dy = wp_y - current_pose_.y;

    float target_yaw = std::atan2(dy, dx);

    execution_yaws_.push_back(target_yaw);

    RCLCPP_INFO(this->get_logger(), "WP[%d] → target yaw=%.4f rad (%.1f deg)",
                idx, target_yaw, target_yaw * 180.0f / M_PI);
  }
}

float PIDMazeSolver::NormalizeAngle(float angle) {
  while (angle > M_PI)
    angle -= 2.0f * M_PI;
  while (angle < -M_PI)
    angle += 2.0f * M_PI;
  return angle;
}

void PIDMazeSolver::StopRobot() {
  // geometry_msgs::msg::Twist cmd;
  Twist cmd;
  cmd.angular.z = 0.0f;
  cmd.linear.x = 0.0f;
  cmd.linear.y = 0.0f;
  twist_pub_->publish(cmd);
}

void PIDMazeSolver::timer_callback() {
  timer_->cancel();
  // Wait until first real odom reading arrives
  rclcpp::Rate wait_rate(10ms);
  while (!initial_odom_received_ && rclcpp::ok()) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "Waiting for odometry...");
    wait_rate.sleep();
  }

  RCLCPP_INFO(this->get_logger(),
              "Live odom ready: pos=(%.4f, %.4f) yaw=%.4f rad", current_pose_.x,
              current_pose_.y, current_pose_.yaw);

  // Build target yaw list with drift compensation
  // BuildExecutionYaws();

  if (execution_yaws_.empty()) {
    RCLCPP_ERROR(this->get_logger(),
                 "No execution yaws — check waypoint_sequence.");
    return;
  }

  // Store initial yaw to return to at the end
  float initial_yaw = current_pose_.yaw;

  rclcpp::Rate rate(100ms); // 10 Hz control loop
  Twist cmd_vel;

  float err_yaw;
  float prev_err_yaw = 0.0f;
  float sum_I = 0.0f;
  float dt;
  rclcpp::Time current_time, prev_time;

  for (size_t i = 0; i < execution_yaws_.size(); i++) {
    float target_yaw = execution_yaws_[i];

    RCLCPP_INFO(this->get_logger(),
                "Turn %zu/%zu → target yaw=%.4f rad (%.1f deg)", i + 1,
                execution_yaws_.size(), target_yaw, target_yaw * 180.0f / M_PI);

    // Reset PID state for each turn
    prev_err_yaw = 0.0f;
    sum_I = 0.0f;
    prev_time = this->get_clock()->now();

    // PID loop until within yaw tolerance
    err_yaw = NormalizeAngle(target_yaw - current_pose_.yaw);
    while (std::abs(err_yaw) >= yaw_tolerance_ && rclcpp::ok()) {

      err_yaw = NormalizeAngle(target_yaw - current_pose_.yaw);
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
      float angular_vel = TkP_ * err_yaw + TkI_ * sum_I + TkD_ * derivative;

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
                           TkP_ * err_yaw, TkI_ * sum_I, TkD_ * derivative,
                           angular_vel);

      prev_err_yaw = err_yaw;
      prev_time = current_time;
      rate.sleep();
    }

    // Stop and pause at each target yaw
    StopRobot();
    RCLCPP_INFO(this->get_logger(),
                "Turn %zu reached | final yaw=%.4f rad | err=%.4f rad", i + 1,
                current_pose_.yaw, err_yaw);
    // current_pose_(2), err_yaw);
    rclcpp::sleep_for(1s);
  }

  RCLCPP_INFO(this->get_logger(), "Returning to initial yaw=%.4f rad",
              initial_yaw);

  prev_err_yaw = 0.0f;
  sum_I = 0.0f;
  prev_time = this->get_clock()->now();

  //   err_yaw = NormalizeAngle(initial_yaw - current_pose_(2));
  err_yaw = NormalizeAngle(initial_yaw - current_pose_.yaw);
  while (std::abs(err_yaw) >= yaw_tolerance_ && rclcpp::ok()) {

    // err_yaw = NormalizeAngle(initial_yaw - current_pose_(2));
    err_yaw = NormalizeAngle(initial_yaw - current_pose_.yaw);
    current_time = this->get_clock()->now();
    dt = (current_time - prev_time).seconds();

    if (dt <= 0.0f) {
      rate.sleep();
      continue;
    }

    sum_I += err_yaw * dt;
    sum_I = std::clamp(sum_I, -1.0f, 1.0f);

    float derivative = (err_yaw - prev_err_yaw) / dt;
    float angular_vel = TkP_ * err_yaw + TkI_ * sum_I + TkD_ * derivative;
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

std::string
PIDMazeSolver::prnt_waypoints(const std::vector<PoseOrient> &vec) const {

  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    oss << "(" << vec[i].x << ", " << vec[i].y << ", " << vec[i].yaw << ")";
    if (i != vec.size() - 1)
      oss << ", ";
  }
  oss << "]";
  return oss.str();
}

void PIDMazeSolver::printWaypointsSequence(const std::vector<size_t> &seq,
                                           const std::string &name) const {
  std::ostringstream oss;
  oss << name << " = [";
  for (size_t i = 0; i < seq.size(); ++i) {
    const PoseOrient &wp = all_waypoints_[seq[i]];
    oss << "(" << wp.x << ", " << wp.y << ", " << wp.yaw << ")";
    if (i != seq.size() - 1) {
      oss << ", ";
    }
  }

  oss << "]";
  RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  int scene_number = 1;
  if (argc > 1) {
    scene_number = std::atoi(argv[1]);
  }

  std::string config_file;

  if (scene_number == 1)
    config_file = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/pid_maze_solver/"
                  "config/sim.yaml";
  else
    config_file = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/pid_maze_solver/"
                  "config/real.yaml";
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", config_file});
  auto node = std::make_shared<PIDMazeSolver>(scene_number, options);

  //   auto node = std::make_shared<PIDMazeSolver>(scene_number);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
