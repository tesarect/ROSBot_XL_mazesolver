#include "pid_maze_solver/pid_maze_solver.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pid_maze_solver/pid.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

// ═════════════════════════════════════════════════════════════════════════════
// Constructor
// ═════════════════════════════════════════════════════════════════════════════
PIDMazeSolver::PIDMazeSolver(int scene_number,
                             const rclcpp::NodeOptions &options)
    : Node("maze_solver_node", options), scene_number_(scene_number),
      tf_buffer_(this->get_clock()) {

  LoadParameters();

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this);

  // ── Subscriptions ─────────────────────────────────────────────────────────
  odom_sub_options_.callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  odom_sub_ = this->create_subscription<Odom>(
      odom_topic_, 10,
      [this](const Odom::SharedPtr msg) {
        current_pose_.x = msg->pose.pose.position.x;
        current_pose_.y = msg->pose.pose.position.y;
        float qz = msg->pose.pose.orientation.z;
        float qw = msg->pose.pose.orientation.w;
        current_pose_.yaw = 2.0f * std::atan2(qz, qw);
        if (!initial_odom_received_) {
          RCLCPP_INFO(get_logger(),
                      "Odom ready — live home: (%.4f, %.4f) yaw=%.4f rad",
                      current_pose_.x, current_pose_.y, current_pose_.yaw);
          initial_odom_received_ = true;
        }
      },
      odom_sub_options_);

  laser_sub_options_.callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  laser_sub_ = this->create_subscription<Laser>(
      laser_topic_, 10,
      std::bind(&PIDMazeSolver::laserCallback, this, std::placeholders::_1),
      laser_sub_options_);

  twist_pub_ = this->create_publisher<Twist>("cmd_vel", 1);

  // ── PID setup ─────────────────────────────────────────────────────────────
  distance_pid_.set_gain(DkP_, DkI_, DkD_);
  distance_pid_.set_limit(max_lin_vel_, 1.0);
  turn_pid_.set_gain(TkP_, TkI_, TkD_);
  turn_pid_.set_limit(max_ang_vel_, 1.0);

  RCLCPP_INFO(get_logger(), "Dist PID: kP=%.3f kI=%.3f kD=%.3f | max_lin=%.3f",
              DkP_, DkI_, DkD_, max_lin_vel_);
  RCLCPP_INFO(get_logger(), "Turn PID: kP=%.3f kI=%.3f kD=%.3f | max_ang=%.3f",
              TkP_, TkI_, TkD_, max_ang_vel_);
  RCLCPP_INFO(get_logger(),
              "Thresholds: front_stop=%.3f wall_nudge=%.3f nudge_gain=%.3f",
              front_stop_thresh_, wall_nudge_thresh_, nudge_gain_);

  // ── Timer: fires once, cancels itself, runs blocking loop ─────────────────
  timer_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ = this->create_wall_timer(
      50ms, std::bind(&PIDMazeSolver::timer_callback, this),
      timer_callback_group_);

  RCLCPP_INFO(get_logger(), "PIDMazeSolver ready ✈️");
}

// ═════════════════════════════════════════════════════════════════════════════
// LoadParameters
// ═════════════════════════════════════════════════════════════════════════════
void PIDMazeSolver::LoadParameters() {
  odom_topic_ =
      this->declare_parameter<std::string>("odom_topic", "/odometry/filtered");
  laser_topic_ = this->declare_parameter<std::string>("laser_topic", "/scan");
  base_link_ = this->declare_parameter<std::string>("base_link", "base_link");
  laser_link_ =
      this->declare_parameter<std::string>("laser_link", "laser_link");

  // Turn PID
  TkP_ = this->declare_parameter<float>("TkP", 3.0f);
  TkI_ = this->declare_parameter<float>("TkI", 0.0001f);
  TkD_ = this->declare_parameter<float>("TkD", 2.0f);
  // Distance PID
  DkP_ = this->declare_parameter<float>("DkP", 0.8f);
  DkI_ = this->declare_parameter<float>("DkI", 0.005f);
  DkD_ = this->declare_parameter<float>("DkD", 0.5f);

  max_ang_vel_ = this->declare_parameter<float>("max_ang_vel", 0.5f);
  max_lin_vel_ = this->declare_parameter<float>("max_lin_vel", 0.3f);
  yaw_tolerance_ = this->declare_parameter<float>("yaw_tolerance", 0.03f);
  goal_tolerance_ = this->declare_parameter<float>("goal_tolerance", 0.03f);

  // ── Laser thresholds ──────────────────────────────────────────────────────
  front_stop_thresh_ =
      this->declare_parameter<float>("front_stop_thresh", 0.25f);
  wall_nudge_thresh_ =
      this->declare_parameter<float>("wall_nudge_thresh", 0.20f);
  nudge_gain_ = this->declare_parameter<float>("nudge_gain", 0.05f);
  front_corr_thresh_ =
      this->declare_parameter<float>("front_corr_thresh", 0.16f);
  side_corr_thresh_ = this->declare_parameter<float>("side_corr_thresh", 0.20f);
  tilt_thresh_ = this->declare_parameter<float>("tilt_thresh", 0.015f);
  tilt_gain_ = this->declare_parameter<float>("tilt_gain", 1.0f);
  correction_timeout_ = this->declare_parameter<int>("correction_timeout", 200);

  RCLCPP_INFO(get_logger(), "Parameters loaded ✅");
}

// ═════════════════════════════════════════════════════════════════════════════
// LoadWaypointsYaml
// Each waypoint = [x, y, yaw] relative delta.
// x/y = metres to move from current pose. yaw = radians to turn from current
// yaw.
// ═════════════════════════════════════════════════════════════════════════════
void PIDMazeSolver::LoadWaypointsYaml() {
  std::string file_path;
  if (scene_number_ == 1 || scene_number_ == 3 || scene_number_ == 5) {
    file_path = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/resources/"
                "waypoints/maze_waypoints_sim.yaml";
  } else if (scene_number_ == 2 || scene_number_ == 4 || scene_number_ == 6) {
    file_path = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/resources"
                "/waypoints/maze_waypoints_real.yaml";
  } else {
    RCLCPP_ERROR(get_logger(), "Invalid scene number: %d", scene_number_);
    return;
  }

  RCLCPP_INFO(get_logger(), "Loading waypoints from: %s", file_path.c_str());

  YAML::Node config;
  try {
    config = YAML::LoadFile(file_path);
  } catch (const YAML::Exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to load YAML: %s", e.what());
    return;
  }

  if (!config["waypoints"] || !config["waypoints"].IsSequence()) {
    RCLCPP_ERROR(get_logger(), "YAML missing 'waypoints' sequence.");
    return;
  }

  waypoints_.clear();
  for (size_t i = 0; i < config["waypoints"].size(); i++) {
    const auto &wp = config["waypoints"][i];
    // Format: [x, y, yaw]
    if (!wp.IsSequence() || wp.size() < 3) {
      RCLCPP_WARN(get_logger(), "WP[%zu] malformed — skipping.", i);
      continue;
    }
    WayPoint w;
    w.x = wp[0].as<float>();
    w.y = wp[1].as<float>();
    w.yaw = wp[2].as<float>();
    waypoints_.push_back(w);
    RCLCPP_INFO(get_logger(),
                "  WP[%zu]: dx=%.4f  dy=%.4f  dyaw=%.4f rad (%.1f deg)", i, w.x,
                w.y, w.yaw, w.yaw * 180.0f / M_PI);
  }

  RCLCPP_INFO(get_logger(), "%zu waypoints loaded.", waypoints_.size());
}

// ═════════════════════════════════════════════════════════════════════════════
// timer_callback — fires once, cancels, then runs the blocking state machine
// ═════════════════════════════════════════════════════════════════════════════
void PIDMazeSolver::timer_callback() {
  timer_->cancel();

  // Wait for first odom and laser
  rclcpp::Rate wait_rate(10);
  while ((!initial_odom_received_ || !initial_laser_received_) &&
         rclcpp::ok()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Waiting for odom=%d laser=%d ...",
                         initial_odom_received_, initial_laser_received_);
    wait_rate.sleep();
  }

  RCLCPP_INFO(get_logger(), "Live pose ready: (%.4f, %.4f) yaw=%.4f rad",
              current_pose_.x, current_pose_.y, current_pose_.yaw);

  LoadWaypointsYaml();

  if (waypoints_.empty()) {
    RCLCPP_ERROR(get_logger(), "No waypoints loaded — aborting.");
    rclcpp::shutdown();
    return;
  }

  // ── State machine loop ────────────────────────────────────────────────────
  while (rclcpp::ok()) {
    switch (state_) {
    case State::INITIAL:
      RCLCPP_INFO(get_logger(), "[INITIAL] Starting maze solver 🚀");
      state_ = State::NEXT;
      break;

    case State::NEXT:
      RCLCPP_INFO(get_logger(),
                  "[NEXT] goal_idx=%zu/%zu | "
                  "Laser N:%.2f E:%.2f W:%.2f S:%.2f",
                  current_goal_idx_, waypoints_.size(), dN, dE, dW, dS);
      get_next_wp();
      break;

    case State::TURN:
      RCLCPP_INFO(get_logger(), "[TURN] goal_idx=%zu", current_goal_idx_);
      face_next_wp();
      break;

    case State::MOVE:
      RCLCPP_INFO(get_logger(), "[MOVE] goal_idx=%zu", current_goal_idx_);
      head_to_next_wp();
      break;

    case State::CORRECTING:
      RCLCPP_INFO(get_logger(), "[CORRECTING] goal_idx=%zu", current_goal_idx_);
      do_correction();
      break;

    case State::FINAL:
      RCLCPP_INFO(get_logger(), "[FINAL] All waypoints complete 🏁");
      publish_vel(0.0, 0.0, 0.0);
      rclcpp::sleep_for(500ms);
      rclcpp::shutdown();
      return;
    }
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// get_next_wp — decide next state based on current goal index
// ═════════════════════════════════════════════════════════════════════════════
void PIDMazeSolver::get_next_wp() {
  if (current_goal_idx_ >= waypoints_.size()) {
    state_ = State::FINAL;
    return;
  }

  // Not yet activated — transition to TURN to begin this waypoint
  if (!goal_active_) {
    state_ = State::TURN;
    return;
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// face_next_wp — rotate by yaw delta from YAML, anchored to current yaw
// ═════════════════════════════════════════════════════════════════════════════
void PIDMazeSolver::face_next_wp() {
  if (current_goal_idx_ >= waypoints_.size()) {
    state_ = State::FINAL;
    return;
  }

  const WayPoint &wp = waypoints_[current_goal_idx_];

  // If no yaw delta, skip turn entirely
  if (std::fabs(wp.yaw) < 0.01f) {
    RCLCPP_INFO(get_logger(), "[TURN] No yaw delta — skipping to MOVE");
    // Capture start pose for MOVE phase
    start_x_ = current_pose_.x;
    start_y_ = current_pose_.y;
    start_yaw_ = current_pose_.yaw;
    goal_active_ = true;
    state_ = State::MOVE;
    return;
  }

  // Anchor target yaw to current yaw + delta
  float target_yaw = NormalizeAngle(current_pose_.yaw + wp.yaw);

  RCLCPP_INFO(get_logger(),
              "[TURN] WP[%zu] | current_yaw=%.4f + delta=%.4f "
              "→ target_yaw=%.4f rad (%.1f deg)",
              current_goal_idx_, current_pose_.yaw, wp.yaw, target_yaw,
              target_yaw * 180.0f / M_PI);

  turn_pid_.reset();
  rclcpp::Rate rate(100);
  rclcpp::Time prev_time = this->get_clock()->now();

  while (rclcpp::ok()) {
    float err_yaw = NormalizeAngle(target_yaw - current_pose_.yaw);

    rclcpp::Time now = this->get_clock()->now();
    float dt = (now - prev_time).seconds();
    prev_time = now;
    if (dt <= 0.0f) {
      rate.sleep();
      continue;
    }

    if (std::fabs(err_yaw) < yaw_tolerance_) {
      publish_vel(0.0, 0.0, 0.0);
      RCLCPP_INFO(get_logger(), "[TURN] Done | final_yaw=%.4f err=%.4f rad",
                  current_pose_.yaw, err_yaw);
      rclcpp::sleep_for(300ms);
      break;
    }

    double ang_vel = turn_pid_.compute(err_yaw, dt);
    publish_vel(0.0, 0.0, ang_vel);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                         "[TURN] err=%.4f rad (%.1f deg) | ang_vel=%.3f",
                         err_yaw, err_yaw * 180.0f / M_PI, ang_vel);
    rate.sleep();
  }

  // Capture start pose for MOVE phase
  start_x_ = current_pose_.x;
  start_y_ = current_pose_.y;
  start_yaw_ = current_pose_.yaw;
  goal_active_ = true;
  state_ = State::MOVE;
}

// ═════════════════════════════════════════════════════════════════════════════
// head_to_next_wp — move by x/y delta anchored to start pose at activation
// Includes:
//   - Wall nudge: push away from side walls if dE or dW too close
//   - Front obstacle early stop: stop if dN < front_stop_thresh_
// ═════════════════════════════════════════════════════════════════════════════
void PIDMazeSolver::head_to_next_wp() {
  if (current_goal_idx_ >= waypoints_.size()) {
    state_ = State::FINAL;
    return;
  }

  const WayPoint &wp = waypoints_[current_goal_idx_];

  // If no movement delta, skip move
  if (std::fabs(wp.x) < 0.001f && std::fabs(wp.y) < 0.001f) {
    RCLCPP_INFO(get_logger(), "[MOVE] No xy delta — skipping to CORRECTING");
    state_ = State::CORRECTING;
    correction_counter_ = 0;
    return;
  }

  // Absolute target anchored to pose at turn completion
  float target_x = start_x_ + wp.x;
  float target_y = start_y_ + wp.y;

  RCLCPP_INFO(get_logger(),
              "[MOVE] WP[%zu] | from=(%.4f, %.4f) + delta=(%.4f, %.4f) "
              "→ target=(%.4f, %.4f)",
              current_goal_idx_, start_x_, start_y_, wp.x, wp.y, target_x,
              target_y);

  distance_pid_.reset();
  rclcpp::Rate rate(100);
  rclcpp::Time prev_time = this->get_clock()->now();
  float prev_dist = std::numeric_limits<float>::max();

  while (rclcpp::ok()) {
    float dx = target_x - current_pose_.x;
    float dy = target_y - current_pose_.y;
    float dist = std::sqrt(dx * dx + dy * dy);

    rclcpp::Time now = this->get_clock()->now();
    float dt = (now - prev_time).seconds();
    prev_time = now;
    if (dt <= 0.0f) {
      rate.sleep();
      continue;
    }

    // ── Goal reached ──────────────────────────────────────────────────────
    if (dist < goal_tolerance_) {
      publish_vel(0.0, 0.0, 0.0);
      RCLCPP_INFO(get_logger(),
                  "[MOVE] Goal reached | dist=%.4f m | pos=(%.4f, %.4f)", dist,
                  current_pose_.x, current_pose_.y);
      rclcpp::sleep_for(300ms);
      state_ = State::CORRECTING;
      correction_counter_ = 0;
      return;
    }

    // ── Front obstacle early stop ─────────────────────────────────────────
    if (dN < static_cast<double>(front_stop_thresh_)) {
      publish_vel(0.0, 0.0, 0.0);
      RCLCPP_WARN(get_logger(),
                  "[MOVE] Front obstacle dN=%.3f < %.3f — stopping early", dN,
                  front_stop_thresh_);
      rclcpp::sleep_for(200ms);
      state_ = State::CORRECTING;
      correction_counter_ = 0;
      return;
    }

    // ── Overshoot guard ───────────────────────────────────────────────────
    if (dist > prev_dist + 0.05f) {
      publish_vel(0.0, 0.0, 0.0);
      RCLCPP_WARN(get_logger(), "[MOVE] Overshoot detected — stopping.");
      rclcpp::sleep_for(500ms);
      state_ = State::CORRECTING;
      correction_counter_ = 0;
      return;
    }
    prev_dist = dist;

    // ── PID velocity in world frame → robot frame ─────────────────────────
    double lin_vel = distance_pid_.compute(dist, dt);

    // Decompose into robot-local x/y using current yaw
    float cos_y = std::cos(current_pose_.yaw);
    float sin_y = std::sin(current_pose_.yaw);
    float local_x = dx * cos_y + dy * sin_y;
    float local_y = -dx * sin_y + dy * cos_y;
    float norm = std::sqrt(local_x * local_x + local_y * local_y);

    double vx = (norm > 0.001f) ? lin_vel * (local_x / norm) : 0.0;
    double vy = (norm > 0.001f) ? lin_vel * (local_y / norm) : 0.0;

    // ── Wall nudge: push away from walls if too close ─────────────────────
    // dW = left wall distance, dE = right wall distance
    if (dW < static_cast<double>(wall_nudge_thresh_)) {
      vy -= nudge_gain_; // too close to left → nudge right
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "[MOVE] Left wall nudge dW=%.3f", dW);
    }
    if (dE < static_cast<double>(wall_nudge_thresh_)) {
      vy += nudge_gain_; // too close to right → nudge left
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "[MOVE] Right wall nudge dE=%.3f", dE);
    }

    publish_vel(vx, vy, 0.0);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                         "[MOVE] dist=%.4f | vx=%.3f vy=%.3f | "
                         "dN=%.2f dE=%.2f dW=%.2f",
                         dist, vx, vy, dN, dE, dW);
    rate.sleep();
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// do_correction — fine-adjust position using laser after each move
// Corrects: front proximity, side proximity, wall tilt
// Exits when all conditions clear or timeout reached
// ═════════════════════════════════════════════════════════════════════════════
void PIDMazeSolver::do_correction() {
  RCLCPP_INFO(get_logger(), "[CORRECTING] Starting corrections...");

  rclcpp::Rate rate(100);

  while (rclcpp::ok()) {
    Twist cmd;
    bool need_correction = false;

    // ── Front proximity — back up ──────────────────────────────────────────
    if (dN < static_cast<double>(front_corr_thresh_)) {
      cmd.linear.x -= 0.03;
      need_correction = true;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                           "[CORRECTING] Front too close dN=%.3f", dN);
    }

    // ── Side proximity — push away ─────────────────────────────────────────
    if (dW < static_cast<double>(side_corr_thresh_)) {
      cmd.linear.y -= 0.03; // left wall → push right
      need_correction = true;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                           "[CORRECTING] Left wall dW=%.3f", dW);
    }
    if (dE < static_cast<double>(side_corr_thresh_)) {
      cmd.linear.y += 0.03; // right wall → push left
      need_correction = true;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                           "[CORRECTING] Right wall dE=%.3f", dE);
    }

    // ── Tilt correction using diagonal pairs ──────────────────────────────
    // Compare closer wall's diagonal readings.
    // If NE > SE → robot nose angled toward right wall → rotate CCW
    // If NW > SW → robot nose angled toward left wall  → rotate CW
    float tilt_corr = 0.0f;
    if (dW < dE) {
      // closer to left wall → use left diagonals
      float diff = static_cast<float>(dNW - dSW);
      if (std::fabs(diff) > tilt_thresh_) {
        tilt_corr = std::clamp(-tilt_gain_ * diff, -0.1f, 0.1f);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "[CORRECTING] Left tilt diff=%.3f → wz=%.3f", diff,
                             tilt_corr);
        need_correction = true;
      }
    } else {
      // closer to right wall → use right diagonals
      float diff = static_cast<float>(dNE - dSE);
      if (std::fabs(diff) > tilt_thresh_) {
        tilt_corr = std::clamp(tilt_gain_ * diff, -0.1f, 0.1f);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "[CORRECTING] Right tilt diff=%.3f → wz=%.3f",
                             diff, tilt_corr);
        need_correction = true;
      }
    }
    cmd.angular.z = tilt_corr;

    if (need_correction) {
      twist_pub_->publish(cmd);
      correction_counter_++;

      if (correction_counter_ > correction_timeout_) {
        RCLCPP_WARN(get_logger(),
                    "[CORRECTING] Timeout (%d cycles) — moving on.",
                    correction_timeout_);
        break;
      }
      rate.sleep();
      continue;
    }

    // All clear
    RCLCPP_INFO(get_logger(), "[CORRECTING] Done after %d cycles.",
                correction_counter_);
    break;
  }

  publish_vel(0.0, 0.0, 0.0);
  rclcpp::sleep_for(300ms);

  // Advance to next waypoint
  current_goal_idx_++;
  goal_active_ = false;
  correction_counter_ = 0;

  if (current_goal_idx_ >= waypoints_.size()) {
    state_ = State::FINAL;
  } else {
    state_ = State::NEXT;
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// align_to_odom_x — rotate robot so base_link X is parallel to odom X
// same_direction=true  → face yaw=0   (same as odom +X)
// same_direction=false → face yaw=π   (opposite direction, -X)
// NOT CALLED — available for manual testing
// ═════════════════════════════════════════════════════════════════════════════
void PIDMazeSolver::align_to_odom_x(bool same_direction) {
  float target_yaw = same_direction ? 0.0f : static_cast<float>(M_PI);

  RCLCPP_INFO(get_logger(),
              "[ALIGN] Aligning to odom X | target_yaw=%.4f rad (%.1f deg) | "
              "same_direction=%s",
              target_yaw, target_yaw * 180.0f / M_PI,
              same_direction ? "true (+X)" : "false (-X)");

  turn_pid_.reset();
  rclcpp::Rate rate(100);
  rclcpp::Time prev_time = this->get_clock()->now();

  while (rclcpp::ok()) {
    float err_yaw = NormalizeAngle(target_yaw - current_pose_.yaw);

    rclcpp::Time now = this->get_clock()->now();
    float dt = (now - prev_time).seconds();
    prev_time = now;
    if (dt <= 0.0f) {
      rate.sleep();
      continue;
    }

    if (std::fabs(err_yaw) < yaw_tolerance_) {
      publish_vel(0.0, 0.0, 0.0);
      RCLCPP_INFO(get_logger(),
                  "[ALIGN] Done | final_yaw=%.4f rad | err=%.4f rad",
                  current_pose_.yaw, err_yaw);
      rclcpp::sleep_for(300ms);
      return;
    }

    double ang_vel = turn_pid_.compute(err_yaw, dt);
    publish_vel(0.0, 0.0, ang_vel);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                         "[ALIGN] err=%.4f rad (%.1f deg) | ang_vel=%.3f",
                         err_yaw, err_yaw * 180.0f / M_PI, ang_vel);
    rate.sleep();
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// initLaser — runs once on first scan, computes direction indices via TF
// ═════════════════════════════════════════════════════════════════════════════
void PIDMazeSolver::initLaser(const Laser::SharedPtr msg) {
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(base_link_, msg->header.frame_id,
                                    msg->header.stamp,
                                    rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "TF lookup failed: %s — retrying", ex.what());
    return;
  }

  tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                    tf.transform.rotation.z, tf.transform.rotation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  laser_to_base_yaw_ = yaw;

  angle_min_ = msg->angle_min;
  angle_increment_ = msg->angle_increment;
  num_rays_ = static_cast<int>(msg->ranges.size());

  auto toIdxBase = [&](double base_deg) {
    double laser_rad = base_deg * M_PI / 180.0 - laser_to_base_yaw_;
    while (laser_rad > M_PI)
      laser_rad -= 2 * M_PI;
    while (laser_rad < -M_PI)
      laser_rad += 2 * M_PI;
    int idx = static_cast<int>(
        std::round((laser_rad - angle_min_) / angle_increment_));
    return std::clamp(idx, 0, num_rays_ - 1);
  };

  north_idx_ = toIdxBase(0.0);
  south_idx_ = toIdxBase(180.0);
  east_idx_ = toIdxBase(90.0);  // right
  west_idx_ = toIdxBase(-90.0); // left
  ne_idx_ = toIdxBase(45.0);
  nw_idx_ = toIdxBase(-45.0);
  se_idx_ = toIdxBase(135.0);
  sw_idx_ = toIdxBase(-135.0);
  front_idx_ = north_idx_;

  band_half_ =
      std::max(1, static_cast<int>(std::round((BAND_DEGREES * M_PI / 180.0) /
                                              angle_increment_)));

  laser_initialized_ = true;
  RCLCPP_INFO(get_logger(),
              "Laser initialized | laser_to_base_yaw=%.4f rad | "
              "band=±%d rays (±%.1f°)",
              laser_to_base_yaw_, band_half_, BAND_DEGREES);
}

// ═════════════════════════════════════════════════════════════════════════════
// laserCallback — updates 8-direction band-averaged ranges + opening detection
// ═════════════════════════════════════════════════════════════════════════════
void PIDMazeSolver::laserCallback(const Laser::SharedPtr msg) {
  if (!laser_initialized_) {
    initLaser(msg);
    return;
  }

  const auto &ranges = msg->ranges;

  dN = bandAvg(ranges, north_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  dNE = bandAvg(ranges, ne_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  dE = bandAvg(ranges, east_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  dSE = bandAvg(ranges, se_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  dS = bandAvg(ranges, south_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  dSW = bandAvg(ranges, sw_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  dW = bandAvg(ranges, west_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  dNW = bandAvg(ranges, nw_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);

  // Opening detection
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
}

// ═════════════════════════════════════════════════════════════════════════════
// bandAvg — average valid ranges within ±half rays of center index
// ═════════════════════════════════════════════════════════════════════════════
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

// ═════════════════════════════════════════════════════════════════════════════
// Utilities
// ═════════════════════════════════════════════════════════════════════════════
void PIDMazeSolver::publish_vel(double vx, double vy, double wz) const {
  Twist cmd;
  cmd.linear.x = vx;
  cmd.linear.y = vy;
  cmd.angular.z = wz;
  twist_pub_->publish(cmd);
}

void PIDMazeSolver::StopRobot() { publish_vel(0.0, 0.0, 0.0); }

bool PIDMazeSolver::isWithinGoalTolerance() const {
  if (current_goal_idx_ >= waypoints_.size())
    return false;
  float dx = current_pose_.x - start_x_ - waypoints_[current_goal_idx_].x;
  float dy = current_pose_.y - start_y_ - waypoints_[current_goal_idx_].y;
  return std::sqrt(dx * dx + dy * dy) <= goal_tolerance_;
}

float PIDMazeSolver::NormalizeAngle(float angle) {
  while (angle > M_PI)
    angle -= 2.0f * M_PI;
  while (angle < -M_PI)
    angle += 2.0f * M_PI;
  return angle;
}

std::string PIDMazeSolver::prnt_waypoints() const {
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < waypoints_.size(); i++) {
    oss << "(dx=" << waypoints_[i].x << " dy=" << waypoints_[i].y
        << " dyaw=" << waypoints_[i].yaw << ")";
    if (i != waypoints_.size() - 1)
      oss << ", ";
  }
  oss << "]";
  return oss.str();
}

// ═════════════════════════════════════════════════════════════════════════════
// main
// ═════════════════════════════════════════════════════════════════════════════
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  int scene_number = 1;
  if (argc > 1)
    scene_number = std::atoi(argv[1]);

  std::string config_file;
  if (scene_number % 2 == 1) {
    config_file = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/"
                  "pid_maze_solver/config/sim.yaml";
  } else {
    config_file = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/"
                  "pid_maze_solver/config/real.yaml";
  }

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", config_file});
  auto node = std::make_shared<PIDMazeSolver>(scene_number, options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}