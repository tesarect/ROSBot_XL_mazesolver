#include "pid_maze_solver/pid_maze_solver.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pid_maze_solver/pid.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "yaml-cpp/yaml.h"
#include <Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nlohmann/json.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sstream>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

using namespace std::chrono_literals;

PIDMazeSolver::PIDMazeSolver(int scene_number,
                             const rclcpp::NodeOptions &options)
    : Node("maze_solver_node", options), scene_number_(scene_number),
      tf_buffer_(this->get_clock()) {
  // PIDMazeSolver::PIDMazeSolver(int scene_number,
  //                              const rclcpp::NodeOptions &options)
  //     : Node("maze_solver_node", options), scene_number_(scene_number) {

  LoadParameters();

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this);

  timer_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ = this->create_wall_timer(
      50ms, std::bind(&PIDMazeSolver::timer_callback, this),
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
        // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1,
        //                      " odom callback ---- 🛰️");
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
              " Dist PID gains: DkP=%.3f DkI=%.3f DkD=%.3f | max_lin_vel=%.3f",
              DkP_, DkI_, DkD_, max_lin_vel_);
  RCLCPP_INFO(this->get_logger(),
              " Turn PID gains: TkP=%.3f TkI=%.3f TkD=%.3f | max_ang_vel=%.3f",
              TkP_, TkI_, TkD_, max_ang_vel_);

  distance_pid_.set_gain(DkP_, DkI_, DkD_);
  distance_pid_.set_limit(max_lin_vel_, 1.0);
  turn_pid_.set_gain(TkP_, TkI_, TkD_);
  turn_pid_.set_limit(max_ang_vel_, 1.0);
  RCLCPP_INFO(this->get_logger(), " Turn & Distance PID set 🎛️");

  RCLCPP_INFO(this->get_logger(),
              "PIDMazeSolver initialization complete. ✈️");

  // std::exit(EXIT_FAILURE); // ⛔  s t o p
}

void PIDMazeSolver::initLaser(const Laser::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), " inside initLaser 🔦 🟢");
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(
        base_link_,           // target  (base_link)
        msg->header.frame_id, // source  (laser frame)
        msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "TF lookup failed: %s — retrying next scan",
                ex.what());
    return; // don't mark initialized, retry on next scan
  }
  tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                    tf.transform.rotation.z, tf.transform.rotation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  laser_to_base_yaw_ = yaw;

  angle_min_ = msg->angle_min;
  angle_increment_ = msg->angle_increment;
  num_rays_ = static_cast<int>(msg->ranges.size());

  //   auto toIdx = [&](double deg) {
  //     int idx = static_cast<int>(
  //         std::round((deg * M_PI / 180.0 - angle_min_) / angle_increment_));
  //     return std::clamp(idx, 0, num_rays_ - 1);
  //   };

  //   if (scene_number_ == 1) {
  //     north_idx_ = toIdx(180.0);
  //     south_idx_ = toIdx(0.0);
  //     east_idx_ = toIdx(90.0);
  //     west_idx_ = toIdx(-90.0);
  //     se_idx_ = toIdx(45.0);
  //     sw_idx_ = toIdx(-45.0);
  //     ne_idx_ = toIdx(135.0);
  //     nw_idx_ = toIdx(-135.0);
  //   } else {
  //     north_idx_ = toIdx(0.0);
  //     south_idx_ = toIdx(180.0);
  //     east_idx_ = toIdx(90.0);
  //     west_idx_ = toIdx(-90.0);
  //     ne_idx_ = toIdx(45.0);
  //     nw_idx_ = toIdx(-45.0);
  //     se_idx_ = toIdx(135.0);
  //     sw_idx_ = toIdx(-135.0);
  //   }

  auto toIdxBase = [&](double base_deg) {
    double laser_rad = base_deg * M_PI / 180.0 - laser_to_base_yaw_;
    // wrap to [angle_min, angle_max]
    while (laser_rad > M_PI)
      laser_rad -= 2 * M_PI;
    while (laser_rad < -M_PI)
      laser_rad += 2 * M_PI;
    int idx = static_cast<int>(
        std::round((laser_rad - angle_min_) / angle_increment_));
    return std::clamp(idx, 0, num_rays_ - 1);
  };

  // These now work for ANY laser mounting — sim or real, 0° or 180° or any
  // angle
  north_idx_ = toIdxBase(0.0);
  south_idx_ = toIdxBase(180.0);
  east_idx_ = toIdxBase(90.0);
  west_idx_ = toIdxBase(-90.0);
  ne_idx_ = toIdxBase(45.0);
  nw_idx_ = toIdxBase(-45.0);
  se_idx_ = toIdxBase(135.0);
  sw_idx_ = toIdxBase(-135.0);

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

  //   RCLCPP_INFO(get_logger(), " inside laserCallback--- 🔦 ");
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

  initial_laser_received_ = true;

  // ── 5. Log openings ─────────────────────────────────────────────────────
  //   for (auto &op : openings_)
  //     RCLCPP_INFO(get_logger(),
  //                 "Opening  center=%.1f°  width=%.1f°  avg_range=%.2f m",
  //                 op.center_angle_deg, op.width_deg, op.avg_range);

  // wall_dist values (dN … dNW) and openings_ are now ready for your
  // maze-solver / motion controller to consume this tick.
}

void PIDMazeSolver::LoadParameters() {
  //   use_sim_time_ = this->declare_parameter<bool>("use_sim_time", false);
  num_waypoints_ = this->declare_parameter<int>("num_waypoints", -1);
  ascending_wp_strafe_ =
      this->declare_parameter<bool>("ascending_wp_strafe", false);
  descending_wp_strafe_ =
      this->declare_parameter<bool>("descending_wp_strafe", false);

  odom_topic_ =
      this->declare_parameter<std::string>("odom_topic", "/odometry/filtered");

  laser_topic_ = this->declare_parameter<std::string>("laser_topic", "/scan");
  base_link_ = this->declare_parameter<std::string>("base_link", "base_link");
  laser_link_ =
      this->declare_parameter<std::string>("laser_link", "laser_link");
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
  max_lin_vel_ = this->declare_parameter<float>("max_lin_vel", 0.5f);

  odom_drift_threshold_ =
      this->declare_parameter<float>("odom_drift_threshold", 0.05f);
  yaw_tolerance_ = this->declare_parameter<float>("yaw_tolerance", 0.05f);
  goal_tolerance_ = this->declare_parameter<float>("goal_tolerance", 0.02f);

  RCLCPP_INFO(this->get_logger(), " -Parameters Initialized");
}

void PIDMazeSolver::LoadWaypointsYaml() {
  // Select file based on scene
  std::string selected_file;

  if (scene_number_ == 1 || scene_number_ == 3 || scene_number_ == 5) {
    selected_file = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/resources/"
                    "waypoints/sim_waypoints.yaml";
  } else if (scene_number_ == 2 || scene_number_ == 4 || scene_number_ == 6) {
    selected_file = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/resources/"
                    "waypoints/real_waypoints.yaml";
    ignore_waypoints_ = {2};
  } else {
    RCLCPP_ERROR(get_logger(), "Invalid scene number");
    return;
  }
  RCLCPP_INFO(get_logger(), " inside LoadWaypointsYaml  🐾");

  load_all_waypoints(selected_file);

  switch (scene_number_) {
  case 1:
    final_waypoint_indices_seq_ = fwd_waypoint_indices_seq_;
    fwd_path_flag_ = true;
    break;
  case 2:
    final_waypoint_indices_seq_ = fwd_waypoint_indices_seq_;
    fwd_path_flag_ = true;
    break;
  case 3:
    if (!fwd_waypoint_indices_seq_.empty()) {
      rev_waypoint_indices_seq_ = fwd_waypoint_indices_seq_;
      std::reverse(rev_waypoint_indices_seq_.begin(),
                   rev_waypoint_indices_seq_.end());

      final_waypoint_indices_seq_ = rev_waypoint_indices_seq_;
      rev_path_flag_ = true;
      printWaypointsSequence(final_waypoint_indices_seq_, "Reverse");
    } else {
      RCLCPP_ERROR(
          get_logger(),
          "Unable to prepare waypoint for simulations reverse waypoints");
      return;
    }
    break;
  case 4:
    if (!fwd_waypoint_indices_seq_.empty()) {
      rev_waypoint_indices_seq_ = fwd_waypoint_indices_seq_;
      std::reverse(rev_waypoint_indices_seq_.begin(),
                   rev_waypoint_indices_seq_.end());
      final_waypoint_indices_seq_ = rev_waypoint_indices_seq_;
      rev_path_flag_ = true;
      printWaypointsSequence(final_waypoint_indices_seq_, "Reverse");
    } else {
      RCLCPP_ERROR(get_logger(),
                   "Unable to prepare waypoint for real reverse waypoints");
      return;
    }
    break;
  case 5:
    if (!fwd_waypoint_indices_seq_.empty()) {
      combined_waypoint_indices_seq_ = fwd_waypoint_indices_seq_;
      combined_waypoint_indices_seq_.insert(
          combined_waypoint_indices_seq_.end(),
          std::next(fwd_waypoint_indices_seq_.rbegin()),
          fwd_waypoint_indices_seq_.rend());
      final_waypoint_indices_seq_ = combined_waypoint_indices_seq_;
      combined_path_flag_ = true;
      printWaypointsSequence(combined_waypoint_indices_seq_, "Combined");
    } else {
      RCLCPP_ERROR(get_logger(),
                   "Unable to prepare waypoint for sim combined waypoints");
      return;
    }
    break;
  case 6:
    if (!fwd_waypoint_indices_seq_.empty()) {
      combined_waypoint_indices_seq_ = fwd_waypoint_indices_seq_;
      combined_waypoint_indices_seq_.insert(
          combined_waypoint_indices_seq_.end(),
          std::next(fwd_waypoint_indices_seq_.rbegin()),
          fwd_waypoint_indices_seq_.rend());
      final_waypoint_indices_seq_ = combined_waypoint_indices_seq_;
      combined_path_flag_ = true;
      printWaypointsSequence(combined_waypoint_indices_seq_, "Combined");
    } else {
      RCLCPP_ERROR(get_logger(),
                   "Unable to prepare waypoint for real combined waypoints");
      return;
    }
    break;
  default:
    RCLCPP_ERROR(this->get_logger(), "Invalid scene number: %d", scene_number_);
    return;
  }

  // Open file
  std::ifstream file(selected_file);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open: %s",
                 selected_file.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), " waypoint sequence selection completed [🐾, 🐾,]");
}

void PIDMazeSolver::load_all_waypoints(const std::string &file_name) {
  RCLCPP_INFO(get_logger(), " all_waypoints [🐾, 🐾, 🐾, 🐾, 🐾, .... ]");

  all_waypoints_.clear();
  YAML::Node yaml_data = YAML::LoadFile(file_name);

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

        bool asc_strafe = entry[key]["ascending_wp_strafe"]
                              ? entry[key]["ascending_wp_strafe"].as<bool>()
                              : false;
        bool desc_strafe = entry[key]["descending_wp_strafe"]
                               ? entry[key]["descending_wp_strafe"].as<bool>()
                               : false;

        // all_waypoints_.push_back(PoseOrient(x, y, yaw, asc_strafe,
        // desc_strafe));
        all_waypoints_.emplace_back(x, y, yaw, asc_strafe, desc_strafe);
        break;
      }
    }
  }

  fwd_waypoint_indices_seq_.clear();
  for (size_t idx = 0; idx < all_waypoints_.size(); ++idx) {
    if (ignore_waypoints_.empty() ||
        std::find(ignore_waypoints_.begin(), ignore_waypoints_.end(), idx) ==
            ignore_waypoints_.end()) {
      fwd_waypoint_indices_seq_.push_back(idx);
    }
  }
  //   RCLCPP_INFO(get_logger(), "Ignore waypoints: %s",
  //             fmt::format("[{}]", fmt::join(ignore_waypoints_, ",
  //             ")).c_str());

  waypoint_sequence_ = all_waypoints_; // shallow copy (safe for struct)

  RCLCPP_INFO(this->get_logger(), " -All Waypoint Loaded");
  RCLCPP_INFO(this->get_logger(), "All waypoints: %s",
              prnt_waypoints(all_waypoints_).c_str());
  RCLCPP_INFO(this->get_logger(), "All waypoints (working copy): %s",
              prnt_waypoints(waypoint_sequence_).c_str());
  printWaypointsSequence(fwd_waypoint_indices_seq_, "Forward");
}

void PIDMazeSolver::ApplyOdomCompensation() {
  if (all_waypoints_.empty())
    RCLCPP_ERROR(get_logger(),
                 "Aborting Odom Compensation as all_waypoints_ are empty");
  return;

  // PoseOrient json_home = all_waypoints_[0];
  PoseOrient json_home = waypoint_sequence_[0];
  PoseOrient live = current_pose_;

  // Position drift
  //   float dx_drift = live.x - json_home.x;
  //   float dy_drift = live.y - json_home.y;
  float dx_drift = current_pose_.x - json_home.x;
  float dy_drift = current_pose_.y - json_home.y;
  float drift = std::sqrt(dx_drift * dx_drift + dy_drift * dy_drift);

  // Yaw offset
  //   float yaw_offset = NormalizeAngle(live.yaw - json_home.yaw);
  float yaw_offset = NormalizeAngle(current_pose_.yaw - json_home.yaw);

  RCLCPP_INFO(get_logger(), " inside applyOdomCompensation 🛰️ ♾️");

  //   RCLCPP_INFO(this->get_logger(),
  //               "JSON home=(%.4f, %.4f, %.4f) | live=(%.4f, %.4f, %.4f)",
  //               json_home.x, json_home.y, json_home.yaw, live.x, live.y,
  //               live.yaw);
  RCLCPP_INFO(this->get_logger(),
              "JSON home=(%.4f, %.4f, %.4f) | live=(%.4f, %.4f, %.4f)",
              json_home.x, json_home.y, json_home.yaw, current_pose_.x,
              current_pose_.y, live.yaw);
  RCLCPP_INFO(this->get_logger(),
              "drift=%.4f m | yaw_offset=%.4f rad | threshold=%.4f m", drift,
              yaw_offset, odom_drift_threshold_);

  if (drift > odom_drift_threshold_) {
    relative_mode_ = true;
    RCLCPP_WARN(this->get_logger(),
                "RELATIVE mode | drift=%.4f m | yaw_offset=%.4f rad", drift,
                yaw_offset);

    // Shift all waypoint positions by drift vector
    // Rotate all waypoint yaws by yaw_offset
    // Modify waypoint_sequence_ (duplicate of `all_waypoints_`)
    for (auto &wp : waypoint_sequence_) {
      // Position compensation
      wp.x += dx_drift;
      wp.y += dy_drift;

      // Yaw compensation — rotate position delta by yaw_offset too
      // (if odom frame is rotated, positions need rotation not just shift)
      if (std::abs(yaw_offset) > 0.087f) { // > 5 degrees
        float cos_y = std::cos(yaw_offset);
        float sin_y = std::sin(yaw_offset);
        // Rotate position around json_home
        float rx = wp.x - json_home.x;
        float ry = wp.y - json_home.y;
        wp.x = json_home.x + rx * cos_y - ry * sin_y + dx_drift;
        wp.y = json_home.y + rx * sin_y + ry * cos_y + dy_drift;
        wp.yaw = NormalizeAngle(wp.yaw + yaw_offset);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Compensated waypoints: %s",
                prnt_waypoints(waypoint_sequence_).c_str());
  } else {
    relative_mode_ = false;
    RCLCPP_INFO(this->get_logger(),
                "ABSOLUTE mode — using JSON positions directly");
  }
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

  RCLCPP_INFO(get_logger(), " inside compute_laser_indices 🔦");
  return dirs;
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
  //   while (!initial_odom_received_ && rclcpp::ok()) {
  while (!initial_odom_received_ && !initial_laser_received_ && rclcpp::ok()) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Waiting for odometry...");
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Waiting for Laser...");
    wait_rate.sleep();
  }

  RCLCPP_INFO(this->get_logger(),
              "Live odom ready: pos=(%.4f, %.4f) yaw=%.4f rad", current_pose_.x,
              current_pose_.y, current_pose_.yaw);

  LoadWaypointsYaml();

  RCLCPP_INFO(get_logger(), " inside timerCallback ⏰ ");

  ApplyOdomCompensation();

  //   RCLCPP_INFO(get_logger(), " Total final waypoint seq : %zu",
  //               final_waypoint_indices_seq_.size());
  total_final_wp_idx_ = final_waypoint_indices_seq_.size() - 1;
  RCLCPP_INFO(get_logger(), " Total final waypoint seq : %zu",
              total_final_wp_idx_);

  while (rclcpp::ok()) {
    switch (state_) {
    case State::INITIAL:
      current_waypoint_index_ = 0;
      next_waypoint_index_ = 0;
      RCLCPP_INFO(
          get_logger(),
          " [INITIAL] state 1️⃣⏩ cur_wp_idx : %zu | nxt_wp_idx : %zu",
          current_waypoint_index_, next_waypoint_index_);
      RCLCPP_INFO(get_logger(),
                  "Dist(m)  N:%.2f  NE:%.2f  E:%.2f  SE:%.2f  S:%.2f  SW:%.2f "
                  "W:%.2f  NW:%.2f",
                  dN, dNE, dE, dSE, dS, dSW, dW, dNW);
      state_ = State::NEXT;
      break;
    case State::NEXT:
      RCLCPP_INFO(
          get_logger(),
          " [NEXT] state 2️⃣⏩ cur_wp_idx : %zu | nxt_wp_idx : %zu",
          current_waypoint_index_, next_waypoint_index_);
      RCLCPP_INFO(get_logger(),
                  "Dist(m)  N:%.2f  NE:%.2f  E:%.2f  SE:%.2f  S:%.2f  SW:%.2f "
                  "W:%.2f  NW:%.2f",
                  dN, dNE, dE, dSE, dS, dSW, dW, dNW);
      get_next_wp();
      break;
    case State::TURN:
      RCLCPP_INFO(
          get_logger(),
          " [TURN] state 3️⃣⏩ cur_wp_idx : %zu | nxt_wp_idx : %zu",
          current_waypoint_index_, next_waypoint_index_);
      RCLCPP_INFO(get_logger(),
                  "Dist(m)  N:%.2f  NE:%.2f  E:%.2f  SE:%.2f  S:%.2f  SW:%.2f "
                  "W:%.2f  NW:%.2f",
                  dN, dNE, dE, dSE, dS, dSW, dW, dNW);
      face_next_wp();
      break;
    case State::MOVE:
      RCLCPP_INFO(
          get_logger(),
          " [MOVE] state 4️⃣⏩ cur_wp_idx : %zu | nxt_wp_idx : %zu",
          current_waypoint_index_, next_waypoint_index_);
      RCLCPP_INFO(get_logger(),
                  "Dist(m)  N:%.2f  NE:%.2f  E:%.2f  SE:%.2f  S:%.2f  SW:%.2f "
                  "W:%.2f  NW:%.2f",
                  dN, dNE, dE, dSE, dS, dSW, dW, dNW);
      head_to_next_wp();
      //   std::exit(EXIT_FAILURE); // ⛔  s t o p
      break;
    case State::FINAL:
      RCLCPP_INFO(
          get_logger(),
          " [FINAL] state 5️⃣⏩ cur_wp_idx : %zu | nxt_wp_idx : %zu",
          current_waypoint_index_, next_waypoint_index_);
      RCLCPP_INFO(get_logger(),
                  "Dist(m)  N:%.2f  NE:%.2f  E:%.2f  SE:%.2f  S:%.2f  SW:%.2f "
                  "W:%.2f  NW:%.2f",
                  dN, dNE, dE, dSE, dS, dSW, dW, dNW);
      //   StopRobot();
      publish_vel(0.0, 0.0, 0.0);
      rclcpp::sleep_for(std::chrono::seconds(1));
      rclcpp::shutdown();
      return;
    }
  }
}

void PIDMazeSolver::get_next_wp() {
  // Guard — check sequence bounds 🚧 END OF FINAL WAYPOINT SEQ Condition
  if (current_waypoint_index_ >= total_final_wp_idx_) {

    RCLCPP_INFO(get_logger(),
                " 🚧 🌀 Current wp idx %zu ▶= final_wp_idx size idx=%zu)",
                current_waypoint_index_, total_final_wp_idx_);
    RCLCPP_INFO(get_logger(), "All waypoints complete.");
    // TODO : state turn. if current wp idx == final_wp_index_seq.size()
    // Face towards previous wp . and the close the loop by final state.
    state_ = State::TURN;
    // state_ = State::FINAL;
    return;
  }

  RCLCPP_INFO(get_logger(),
              " 🌀 Current wp idx %zu ◀= final_wp_idx size idx=%zu)",
              current_waypoint_index_, total_final_wp_idx_);

  // Current target waypoint
  size_t wp_idx = final_waypoint_indices_seq_[current_waypoint_index_];
  const PoseOrient &current_wp = waypoint_sequence_[wp_idx];

  if (isWithinGoalTolerance(current_wp)) {
    RCLCPP_INFO(get_logger(), "✅ Reached waypoint %zu (seq idx=%zu)",
                current_waypoint_index_, wp_idx);

    // Prepare next waypoint
    next_waypoint_index_ = current_waypoint_index_ + 1;

    //  can we use strafe here to decide if we need TURN and skip it ?
    // Check if more waypoints remain
    if (strafe()) {
      state_ = State::MOVE;
    } else if (current_waypoint_index_ >= total_final_wp_idx_) {
      state_ = State::FINAL;
    } else {
      state_ = State::TURN; // face next waypoint
    }
  } else {
    // TODO: DOUBLE CHECK IF THIS CONTION EVER OCCURS IN BOTH SIM AND REAL
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "⚠️ Not at waypoint %zu yet — distance off",
                         current_waypoint_index_);
    state_ = State::TURN; // try to navigate there
    return;               // ⚠️ check if its needed
  }
}

void PIDMazeSolver::face_next_wp() {

  bool _face_previous_wp_ = false;
  size_t target_idx;
  PoseOrient target;
  if (current_waypoint_index_ == total_final_wp_idx_) {

    RCLCPP_INFO(get_logger(),
                " 🚧 🌀 Current wp idx %zu 🟰 final_wp_idx size idx=%zu)",
                current_waypoint_index_, total_final_wp_idx_);
    RCLCPP_INFO(get_logger(),
                "All waypoints complete - facing towards previous WayPoint");
    current_waypoint_index_--;
    target_idx = final_waypoint_indices_seq_[current_waypoint_index_];
    target = waypoint_sequence_[target_idx];
    printWaypointStruct(target,
                        "[FINAL TURN] 🚧 ✋ Next Waypoint - should be previous");
    _face_previous_wp_ = true;
    // TODO : state turn. if current wp idx == final_wp_index_seq.size()
    // Face towards previous wp . and the close the loop by final state.
    // state_ = State::TURN
    //  return;
  } else {
    target_idx = final_waypoint_indices_seq_[next_waypoint_index_];
    target = waypoint_sequence_[target_idx];
    printWaypointStruct(target, "[TURN] Next Waypoint");
  }
  //   if (face_previous_wp_) {
  //     size_t target_idx =
  //     final_waypoint_indices_seq_[current_waypoint_index_--];
  //   } else {
  //     size_t target_idx = final_waypoint_indices_seq_[next_waypoint_index_];
  //   }

  // Compute angle from current position toward next waypoint
  float dx = target.x - current_pose_.x;
  float dy = target.y - current_pose_.y;
  float target_yaw = std::atan2(dy, dx);

  RCLCPP_INFO(this->get_logger(),
              "[TURN] WP[%zu] → target_yaw=%.4f rad (%.1f deg) | "
              "current_yaw=%.4f rad",
              target_idx, target_yaw, target_yaw * 180.0f / M_PI,
              current_pose_.yaw);

  turn_pid_.reset();
  float err_yaw = NormalizeAngle(target_yaw - current_pose_.yaw);

  rclcpp::Rate rate(100ms);
  rclcpp::Time prev_time = this->get_clock()->now();
  geometry_msgs::msg::Twist cmd;

  while (std::abs(err_yaw) >= yaw_tolerance_ && rclcpp::ok()) {
    err_yaw = NormalizeAngle(target_yaw - current_pose_.yaw);

    rclcpp::Time now = this->get_clock()->now();
    float dt = (now - prev_time).seconds();
    prev_time = now;

    if (dt <= 0.0f) {
      rate.sleep();
      continue;
    }

    double ang_vel = turn_pid_.compute(err_yaw, dt);
    publish_vel(0.0, 0.0, ang_vel);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "[TURN] err=%.4f rad (%.1f deg) | ang_vel=%.3f",
                         err_yaw, err_yaw * 180.0f / M_PI, ang_vel);

    rate.sleep();
  }

  //   StopRobot();
  publish_vel(0.0, 0.0, 0.0);
  rclcpp::sleep_for(std::chrono::seconds(1));
  RCLCPP_INFO(this->get_logger(),
              "[TURN] Done | final_yaw=%.4f rad | err=%.4f rad",
              current_pose_.yaw, err_yaw);
  // TODO: LOG Facing next wp

  // Transition to MOVE/FINAL state
  if (_face_previous_wp_) {
    state_ = State::FINAL;
  RCLCPP_INFO(this->get_logger(),
              "[FINAL TURN] Final state Set 🏁");
    return;
  }
  state_ = State::MOVE;
  return; // ⚠️ check if its needed
}

void PIDMazeSolver::head_to_next_wp() {
  RCLCPP_INFO(get_logger(),
              "[MOVE] PID gains: kP=%.3f kI=%.3f kD=%.3f | limit=%.3f", DkP_,
              DkI_, DkD_, max_lin_vel_);
  size_t target_idx = final_waypoint_indices_seq_[next_waypoint_index_];
  //   PoseOrient target = waypoint_sequence_[target_idx];
  const PoseOrient &target = waypoint_sequence_[target_idx];
  printWaypointStruct(target, "Next Waypoint");

  // Compute angle from current position toward next waypoint
  float dx = target.x - current_pose_.x;
  float dy = target.y - current_pose_.y;

  float target_dist = std::sqrt(dx * dx + dy * dy);
  double prev_target_dist = target_dist;
  RCLCPP_INFO(get_logger(),
              "[MOVE] WP[%zu] → target=(%.4f, %.4f) | current=(%.4f, %.4f) | "
              "target_dist=%.4f m",
              target_idx, target.x, target.y, current_pose_.x, current_pose_.y,
              target_dist);

  distance_pid_.reset();
  //   rclcpp::Rate rate(100ms);
  rclcpp::Rate rate(100);
  rclcpp::Time prev_time = this->get_clock()->now();

  while (target_dist >= goal_tolerance_ && rclcpp::ok()) {
    // Recompute distance each iteration from fresh odom
    dx = target.x - current_pose_.x;
    dy = target.y - current_pose_.y;
    target_dist = std::sqrt(dx * dx + dy * dy);

    rclcpp::Time now = this->get_clock()->now();
    float dt = (now - prev_time).seconds();
    prev_time = now;

    if (dt <= 0.0f) {
      rate.sleep();
      continue;
    }

    double lin_vel = distance_pid_.compute(target_dist, dt);
    const double overshoot_margin = 0.05;

    if (target_dist > prev_target_dist + overshoot_margin && lin_vel > 0.05) {
      // if (target_dist > prev_target_dist + overshoot_margin) {
      RCLCPP_WARN(get_logger(), "[MOVE] 🚨 🚨 🚨 Overshoot detected. Stopping.");
      publish_vel(0.0, 0.0, 0.0);
      rclcpp::sleep_for(std::chrono::seconds(1));
      break;
    }
    prev_target_dist = target_dist;

    if (strafe()) {
      float robot_yaw = current_pose_.yaw;
      float local_x = dx * std::cos(robot_yaw) + dy * std::sin(robot_yaw);
      float local_y = -dx * std::sin(robot_yaw) + dy * std::cos(robot_yaw);
      float norm = std::sqrt(local_x * local_x + local_y * local_y);

      double vx = (norm > 0.001f) ? lin_vel * (local_x / norm) : 0.0;
      double vy = (norm > 0.001f) ? lin_vel * (local_y / norm) : 0.0;

      publish_vel(vx, vy, 0.0); // pure strafe, no rotation

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "[STRAFE-MOVE] target_dist=%.3f | vx=%.3f vy=%.3f",
                           target_dist, vx, vy);
    } else {
      publish_vel(lin_vel, 0.0, 0.0);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "[MOVE] target_dist=%.4f m | lin_vel=%.3f",
                           target_dist, lin_vel);
    }

    rate.sleep();
  }

  //   StopRobot();
  publish_vel(0.0, 0.0, 0.0);
  rclcpp::sleep_for(std::chrono::seconds(1));
  RCLCPP_INFO(get_logger(),
              "[MOVE] Done | final target_dist=%.4f m | pos=(%.4f, %.4f)",
              target_dist, current_pose_.x, current_pose_.y);

  // Advance waypoint index and go back to INITIAL to check arrival
  current_waypoint_index_ = next_waypoint_index_;
  // state_ = State::INITIAL;
  state_ = State::NEXT;
  return; // ⚠️ check if its needed
}

// change the function name to prep if any other functionality is added
bool PIDMazeSolver::strafe() const {
  bool strafe_ = false;
  if (fwd_path_flag_) {
    strafe_ =
        waypoint_sequence_[final_waypoint_indices_seq_[current_waypoint_index_]]
            .ascending_wp_strafe;
  } else if (rev_path_flag_) {
    strafe_ =
        waypoint_sequence_[final_waypoint_indices_seq_[current_waypoint_index_]]
            .descending_wp_strafe;
  } else {
    size_t mid = final_waypoint_indices_seq_.size() / 2;
    if (current_waypoint_index_ > mid) {
      // Returning back path
      strafe_ = waypoint_sequence_
                    [final_waypoint_indices_seq_[current_waypoint_index_]]
                        .descending_wp_strafe;
    } else {
      // Heading to Finish point (forward path)
      strafe_ = waypoint_sequence_
                    [final_waypoint_indices_seq_[current_waypoint_index_]]
                        .ascending_wp_strafe;
    }
  }
  return strafe_;
}

void PIDMazeSolver::publish_vel(double vx, double vy, double wz) const {
  // TODO: should we convert vx, vy, qz to &vx, &vy, &qz
  Twist cmd;
  cmd.linear.x = vx;
  cmd.linear.y = vy;
  cmd.angular.z = wz;
  //   pub_->publish(cmd);
  twist_pub_->publish(cmd);
}

bool PIDMazeSolver::isWithinGoalTolerance(const PoseOrient &target) const {
  double dx = current_pose_.x - target.x;
  double dy = current_pose_.y - target.y;

  double distance = std::sqrt(dx * dx + dy * dy);
  return (distance <= goal_tolerance_);
}

std::string
PIDMazeSolver::prnt_waypoints(const std::vector<PoseOrient> &vec) const {
  std::ostringstream oss;
  oss << std::boolalpha;
  oss << "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    oss << "(" << vec[i].x << ", " << vec[i].y << ", " << vec[i].yaw
        << ", asc_stf:" << vec[i].ascending_wp_strafe
        << ", des_stf:" << vec[i].descending_wp_strafe << ")";
    if (i != vec.size() - 1)
      oss << ", ";
  }
  oss << "]";
  return oss.str();
}

void PIDMazeSolver::printWaypointsSequence(const std::vector<size_t> &seq,
                                           const std::string &name) const {
  std::ostringstream oss;
  oss << std::boolalpha;
  oss << name << " = [";
  for (size_t i = 0; i < seq.size(); ++i) {
    const PoseOrient &wp = waypoint_sequence_[seq[i]];
    oss << "(" << wp.x << ", " << wp.y << ", " << wp.yaw
        << ", asc_stf:" << waypoint_sequence_[seq[i]].ascending_wp_strafe
        << ", des_stf:" << waypoint_sequence_[seq[i]].descending_wp_strafe
        << ")";
    if (i != seq.size() - 1) {
      oss << ", ";
    }
  }

  oss << "]";
  RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
}

void PIDMazeSolver::printWaypointStruct(const PoseOrient &wp,
                                        const std::string &name) const {
  std::ostringstream oss;
  oss << std::boolalpha;

  oss << name << " = { "
      << "x: " << wp.x << " | y: " << wp.y << " | yaw: " << wp.yaw
      << " | asc_stf: " << wp.ascending_wp_strafe
      << " | des_stf: " << wp.descending_wp_strafe << " }";

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

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
