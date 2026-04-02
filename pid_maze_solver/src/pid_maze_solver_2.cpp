// ═══════════════════════════════════════════════════════════════════════════
// pid_maze_solver_v4.cpp
//
// A faithful port of the reference implementation (other_3_pid_maze_solver)
// with our project's parameter-file and waypoint-loading structure.
//
// What is his logic exactly:
//   1.  Timer starts only after first odom arrives (not at node construction).
//       This avoids the control loop firing before any state is known.
//
//   2.  Two-phase per waypoint: TURN first, then MOVE.
//       Each waypoint carries (dx, dy, dyaw).
//       dyaw is always executed as a pure rotation before any translation.
//
//   3.  start_segment():
//       - target_yaw  = current_yaw + dyaw
//       - target_x/y  = current_pos + (dx,dy) rotated by target_yaw
//       So dx/dy are expressed in the robot frame AFTER the turn.
//
//   4.  TURN PID:
//       - error     = normalize(target_yaw - current_yaw)
//       - D term    = 0 - imu_yaw_rate   ← uses IMU angular velocity directly,
//                     much cleaner than differencing yaw from odom
//       - done when |e_yaw| < angular_tolerance AND |imu_rate| < ang_vel_tol
//
//   5.  MOVE PID:
//       - world-frame error (ex, ey) rotated into robot frame (ex_b, ey_b)
//       - D term    = 0 - current_twist.linear.x/y  ← uses odom velocity,
//                     cleaner than (err - prev_err) / dt
//       - done when dist < linear_tolerance AND |vel| < linear_vel_tolerance
//
//   6.  apply_wall_avoidance():
//       - computes motion direction from current cmd (atan2(vy, vx))
//       - scans all beams within ±30° of that direction
//       - hard stop if closest beam < stop_distance
//       - proportional slowdown if closest beam < slow_distance
//       - filters out antenna self-echo behind the robot
//
//   7.  anti_drift:
//       - at end of each MOVE, measures how far yaw drifted from target
//       - if drift > 5°, adds it to the NEXT waypoint's dyaw
//       - this prevents accumulated heading error across waypoints
//
//   8.  Pause between waypoints:
//       - implemented as a tick counter (no sleep_for, no blocking)
//       - pause_duration_sec_ * rate_hz_ ticks of doing nothing
//
// Parameter loading: via --params-file passed in main()
// Waypoint loading:  via yaml-cpp, path from ament_index
// ═══════════════════════════════════════════════════════════════════════════

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/logging.hpp"
#include "yaml-cpp/yaml.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <vector>

using namespace std::chrono_literals;

// ─── Waypoint ────────────────────────────────────────────────────────────────
// One entry in the YAML waypoints list.
// Format in YAML: [dx, dy, dyaw]
//   dx   – forward  distance in robot frame AFTER the turn  (metres)
//   dy   – lateral  distance in robot frame AFTER the turn  (metres)
//   dyaw – yaw delta to execute BEFORE the translation      (radians)
struct Waypoint {
  double dx{0.0};
  double dy{0.0};
  double dyaw{0.0};
};

// ═══════════════════════════════════════════════════════════════════════════
class PIDMazeSolver : public rclcpp::Node {
public:
  // Constructor takes scene_number and NodeOptions (which carry --params-file).
  PIDMazeSolver(int scene_number, const rclcpp::NodeOptions &options)
      : Node("maze_solver_node", options), scene_number_(scene_number) {

    // ── 1. Declare & read all ROS parameters ────────────────────────────
    // All values come from the YAML config file passed via --params-file.
    // Defaults here are fallbacks only; normally the config file overrides.

    // Topics
    declare_parameter("odom_topic", "/odometry/filtered");
    declare_parameter("laser_topic", "/scan");
    declare_parameter("imu_topic", "imu_broadcaster/imu");

    // Waypoint loading
    declare_parameter("num_waypoints", -1); // -1 = load all

    // Distance PID  (x and y use the same gains — separate structs so we
    // could tune them independently later if needed)
    declare_parameter("pid_x_kp", 2.1);
    declare_parameter("pid_x_ki", 0.001);
    declare_parameter("pid_x_kd", 0.3);

    declare_parameter("pid_y_kp", 2.1);
    declare_parameter("pid_y_ki", 0.001);
    declare_parameter("pid_y_kd", 0.3);

    // Yaw PID
    declare_parameter("pid_yaw_kp", 1.25);
    declare_parameter("pid_yaw_ki", 0.001);
    declare_parameter("pid_yaw_kd", 0.3);

    // Velocity limits
    declare_parameter("max_linear_vel", 0.45);
    declare_parameter("max_angular_vel", 1.4);

    // Goal tolerances
    declare_parameter("linear_tolerance", 0.01);      // metres
    declare_parameter("angular_tolerance", 0.01);     // rad
    declare_parameter("linear_vel_tolerance", 0.01);  // m/s
    declare_parameter("angular_vel_tolerance", 0.05); // rad/s

    // Pause between waypoints (no sleep_for — just skip N ticks)
    declare_parameter("pause_duration_sec", 1.5); // seconds

    // Obstacle avoidance
    declare_parameter("stop_distance", 0.175);  // hard-stop (metres)
    declare_parameter("slow_distance", 0.5);    // start slowdown (metres)
    declare_parameter("front_angle_deg", 30.0); // ±deg sector

    // Laser mounting offset (180° = laser faces backwards on ROSBot XL)
    declare_parameter("laser_yaw_deg", 180.0);

    // Antenna self-echo filter
    // The real ROSBot XL has a rear antenna that reflects the lidar beam.
    // Beams near antenna_center within antenna_half_width that are closer
    // than antenna_max_range are ignored as self-echoes.
    declare_parameter("antenna_center_deg", -170.0);
    declare_parameter("antenna_half_width_deg", 9.0); // ≈ ±9°
    declare_parameter("antenna_max_range", 0.21);

    // Anti-drift
    // After each MOVE, if yaw drifted more than this threshold from target,
    // inject the drift into the next waypoint's dyaw to compensate.
    declare_parameter("anti_drift", true);
    declare_parameter("yaw_drift_threshold_deg", 5.0);

    // ── Read all parameters into member variables ────────────────────────
    odom_topic_ = get_parameter("odom_topic").as_string();
    laser_topic_ = get_parameter("laser_topic").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();

    num_waypoints_ = get_parameter("num_waypoints").as_int();

    pid_x_.kp = get_parameter("pid_x_kp").as_double();
    pid_x_.ki = get_parameter("pid_x_ki").as_double();
    pid_x_.kd = get_parameter("pid_x_kd").as_double();

    pid_y_.kp = get_parameter("pid_y_kp").as_double();
    pid_y_.ki = get_parameter("pid_y_ki").as_double();
    pid_y_.kd = get_parameter("pid_y_kd").as_double();

    pid_yaw_.kp = get_parameter("pid_yaw_kp").as_double();
    pid_yaw_.ki = get_parameter("pid_yaw_ki").as_double();
    pid_yaw_.kd = get_parameter("pid_yaw_kd").as_double();

    max_linear_vel_ = get_parameter("max_linear_vel").as_double();
    max_angular_vel_ = get_parameter("max_angular_vel").as_double();

    linear_tolerance_ = get_parameter("linear_tolerance").as_double();
    angular_tolerance_ = get_parameter("angular_tolerance").as_double();
    linear_vel_tolerance_ = get_parameter("linear_vel_tolerance").as_double();
    angular_vel_tolerance_ = get_parameter("angular_vel_tolerance").as_double();

    pause_duration_sec_ = get_parameter("pause_duration_sec").as_double();

    stop_distance_ = get_parameter("stop_distance").as_double();
    slow_distance_ = get_parameter("slow_distance").as_double();
    front_angle_rad_ =
        get_parameter("front_angle_deg").as_double() * M_PI / 180.0;

    laser_yaw_in_base_ =
        get_parameter("laser_yaw_deg").as_double() * M_PI / 180.0;

    antenna_center_ =
        get_parameter("antenna_center_deg").as_double() * M_PI / 180.0;
    antenna_half_width_ =
        get_parameter("antenna_half_width_deg").as_double() * M_PI / 180.0;
    antenna_max_range_ = get_parameter("antenna_max_range").as_double();

    anti_drift_ = get_parameter("anti_drift").as_bool();
    yaw_drift_threshold_ =
        get_parameter("yaw_drift_threshold_deg").as_double() * M_PI / 180.0;

    // Derived
    pause_ticks_goal_ = static_cast<int>(pause_duration_sec_ * rate_hz_);

    // ── 2. Load waypoints from YAML ──────────────────────────────────────
    waypoints_ = load_waypoints();
    if (waypoints_.empty()) {
      RCLCPP_FATAL(get_logger(), "No waypoints loaded — shutting down.");
      rclcpp::shutdown();
      return;
    }
    // Optionally limit to first N waypoints
    if (num_waypoints_ > 0 &&
        num_waypoints_ < static_cast<int>(waypoints_.size())) {
      waypoints_.resize(num_waypoints_);
      RCLCPP_INFO(get_logger(), "Limiting to first %d waypoints.",
                  num_waypoints_);
    }

    // ── 3. ROS subscribers / publisher ──────────────────────────────────
    // Reentrant callback group so odom, scan, and imu can fire in parallel.
    callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = callback_group_;

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&PIDMazeSolver::odom_callback, this, std::placeholders::_1),
        sub_opts);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        laser_topic_, 10,
        std::bind(&PIDMazeSolver::scan_callback, this, std::placeholders::_1),
        sub_opts);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 10,
        std::bind(&PIDMazeSolver::imu_callback, this, std::placeholders::_1),
        sub_opts);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(get_logger(),
                "PIDMazeSolver v4 ready (scene=%d). "
                "Waiting for first odom to start control loop...",
                scene_number_);

    RCLCPP_INFO(get_logger(),
                "PID x(%.2f,%.4f,%.2f) y(%.2f,%.4f,%.2f) "
                "yaw(%.2f,%.4f,%.2f) | v_max=%.2f w_max=%.2f",
                pid_x_.kp, pid_x_.ki, pid_x_.kd, pid_y_.kp, pid_y_.ki,
                pid_y_.kd, pid_yaw_.kp, pid_yaw_.ki, pid_yaw_.kd,
                max_linear_vel_, max_angular_vel_);
  }

private:
  // ── ROS handles ──────────────────────────────────────────────────────────
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // ── Config ────────────────────────────────────────────────────────────────
  int scene_number_{1};
  std::string odom_topic_, laser_topic_, imu_topic_;
  int num_waypoints_{-1};

  // PID gains struct  (same pattern as original)
  struct PIDGains {
    double kp{0.0}, ki{0.0}, kd{0.0};
  };
  PIDGains pid_x_, pid_y_, pid_yaw_;

  double max_linear_vel_{0.45};
  double max_angular_vel_{1.4};
  double linear_tolerance_{0.01};
  double angular_tolerance_{0.01};
  double linear_vel_tolerance_{0.01};
  double angular_vel_tolerance_{0.05};

  // Control rate — fixed at 20 Hz (same as original)
  // dt_ is used in the integrator: sum_I += error * dt_
  const double rate_hz_{20.0};
  const double dt_{1.0 / rate_hz_};

  // Pause between waypoints
  double pause_duration_sec_{1.5};
  int pause_ticks_goal_{0}; // computed in constructor: duration * rate_hz

  // Obstacle avoidance
  double laser_yaw_in_base_{M_PI}; // 180° for ROSBot XL
  double front_angle_rad_{30.0 * M_PI / 180.0};
  double stop_distance_{0.175};
  double slow_distance_{0.5};

  // Antenna self-echo filter
  double antenna_center_{-170.0 * M_PI / 180.0};
  double antenna_half_width_{9.0 * M_PI / 180.0};
  double antenna_max_range_{0.21};

  // Anti-drift
  bool anti_drift_{true};
  double yaw_drift_threshold_{5.0 * M_PI / 180.0};

  // ── Runtime state ─────────────────────────────────────────────────────────

  // Phase: always TURN first, then MOVE, for every waypoint
  enum class Phase { TURN, MOVE };
  Phase phase_{Phase::TURN};

  // Odom state  (updated in odom_callback)
  bool odom_received_{false};
  double current_x_{0.0}, current_y_{0.0}, current_yaw_{0.0};
  geometry_msgs::msg::Twist current_twist_; // holds linear.x/y velocities

  // IMU state  (updated in imu_callback)
  double imu_yaw_rate_{0.0}; // rad/s — used as D-term input for turn PID

  // Latest laser scan  (updated in scan_callback, used in apply_wall_avoidance)
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  // Waypoint list and current index
  std::vector<Waypoint> waypoints_;
  size_t current_idx_{0};

  // Whether a segment (turn + move) is currently active
  bool segment_active_{false};

  // World-frame targets for current segment  (set in start_segment)
  double target_x_{0.0}, target_y_{0.0}, target_yaw_{0.0};

  // PID integrators  (reset at start of each segment)
  double sum_I_x_{0.0}, sum_I_y_{0.0}, sum_I_yaw_{0.0};

  // Command built each tick and published at the end of control_loop
  geometry_msgs::msg::Twist cmd_;

  // Pause state
  bool pausing_{false};
  int pause_ticks_{0};

  // ── Callbacks ─────────────────────────────────────────────────────────────

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Update position and heading from odometry
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    current_yaw_ = tf2::getYaw(q);

    // Keep body-frame velocities — used as the D-term input in MOVE PID
    // (desired velocity = 0, so error = 0 - current_twist)
    current_twist_ = msg->twist.twist;

    // Start the control loop timer on the FIRST odom message.
    // This guarantees we never tick the loop before state is available.
    if (!odom_received_) {
      odom_received_ = true;
      RCLCPP_INFO(get_logger(),
                  "First odom: x=%.3f y=%.3f yaw=%.3f — starting control loop.",
                  current_x_, current_y_, current_yaw_);

      control_timer_ = create_wall_timer(
          std::chrono::duration<double>(dt_),
          std::bind(&PIDMazeSolver::control_loop, this), callback_group_);
    }
  }

  // Laser scan just stored — used in apply_wall_avoidance each control tick
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = msg;
  }

  // IMU angular velocity — used directly as D-term for turn PID
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_yaw_rate_ = msg->angular_velocity.z;
  }

  // ── Waypoint loading ──────────────────────────────────────────────────────

  std::vector<Waypoint> load_waypoints() {
    // Resolve package share directory so the binary finds waypoints regardless
    // of where it is launched from.
    std::string pkg_dir =
        ament_index_cpp::get_package_share_directory("pid_maze_solver");

    // Map scene number to waypoint filename (same mapping as our earlier code)
    std::string filename;
    switch (scene_number_) {
    case 1:
      filename = "waypoints_sim.yaml";
      break;
    case 2:
      filename = "waypoints_real.yaml";
      break;
    case 3:
      filename = "rev_waypoints_sim.yaml";
      break;
    case 4:
      filename = "rev_waypoints_real.yaml";
      break;
    default:
      RCLCPP_FATAL(get_logger(), "Invalid scene number: %d", scene_number_);
      return {};
    }

    std::string path = pkg_dir + "/waypoints/" + filename;
    RCLCPP_INFO(get_logger(), "Loading waypoints from: %s", path.c_str());

    std::vector<Waypoint> wps;
    try {
      YAML::Node cfg = YAML::LoadFile(path);

      if (!cfg["waypoints"]) {
        RCLCPP_ERROR(get_logger(), "No 'waypoints' key in %s", path.c_str());
        return {};
      }

      for (const auto &node : cfg["waypoints"]) {
        // Each entry must be a 3-element sequence: [dx, dy, dyaw]
        if (!node.IsSequence() || node.size() != 3) {
          RCLCPP_ERROR(get_logger(),
                       "Bad waypoint entry in %s — expected [dx, dy, dyaw]",
                       path.c_str());
          continue;
        }
        Waypoint wp;
        wp.dx = node[0].as<double>();
        wp.dy = node[1].as<double>();
        wp.dyaw = node[2].as<double>();
        wps.push_back(wp);
      }

      RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from %s", wps.size(),
                  filename.c_str());
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(get_logger(), "YAML error loading %s: %s", path.c_str(),
                   e.what());
    }
    return wps;
  }

  // ── Control loop (20 Hz) ─────────────────────────────────────────────────

  void control_loop() {

    // ── Pause between waypoints ─────────────────────────────────────────
    // Instead of sleep_for (which blocks the timer thread and breaks dt),
    // we count ticks and do nothing during the pause window.
    if (pausing_) {
      stop();
      pause_ticks_++;
      if (pause_ticks_ >= pause_ticks_goal_)
        pausing_ = false;
      return;
    }

    // ── All waypoints done ──────────────────────────────────────────────
    if (current_idx_ >= waypoints_.size()) {
      stop();
      RCLCPP_INFO(get_logger(), "All waypoints complete. Done!");
      control_timer_->cancel();
      rclcpp::shutdown();
      return;
    }

    // ── Activate a new segment if none is running ───────────────────────
    if (!segment_active_)
      start_segment();

    // ── TURN phase ──────────────────────────────────────────────────────
    if (phase_ == Phase::TURN) {

      // Heading error: how far we are from the desired yaw
      double e_yaw = normalize_angle(target_yaw_ - current_yaw_);

      // D-term: uses IMU yaw rate directly.
      // We want yaw rate = 0 at goal, so de_yaw = 0 - imu_yaw_rate_.
      // IMU is cleaner than differencing odom yaw, which is noisy.
      double de_yaw = 0.0 - imu_yaw_rate_;

      // Turn is done when both angle error AND angular velocity are small.
      // Checking velocity prevents stopping prematurely while still spinning.
      if (std::abs(e_yaw) < angular_tolerance_ &&
          std::abs(imu_yaw_rate_) < angular_vel_tolerance_) {
        RCLCPP_INFO(get_logger(),
                    "Segment %zu: TURN done (e_yaw=%.4f rad). → MOVE",
                    current_idx_, e_yaw);
        sum_I_yaw_ = 0.0;
        phase_ = Phase::MOVE;
        stop();
        return;
      }

      compute_turn_pid(e_yaw, de_yaw);

      // ── MOVE phase ──────────────────────────────────────────────────────
    } else {

      // World-frame position error
      double ex = target_x_ - current_x_;
      double ey = target_y_ - current_y_;

      // Rotate world-frame error into robot (base_link) frame.
      // PID outputs are robot-frame velocities (vx = forward, vy = left).
      double ex_b = std::cos(current_yaw_) * ex + std::sin(current_yaw_) * ey;
      double ey_b = -std::sin(current_yaw_) * ex + std::cos(current_yaw_) * ey;

      // D-term inputs: desired velocity = 0, so error = 0 - actual_velocity.
      // Uses odom twist (body frame) — cleaner than differencing position.
      double dex = 0.0 - current_twist_.linear.x;
      double dey = 0.0 - current_twist_.linear.y;

      double dist = std::hypot(ex_b, ey_b);

      // Measure yaw drift from target — used for anti-drift injection
      double yaw_drift = normalize_angle(target_yaw_ - current_yaw_);

      // Move is done when both position and velocity are within tolerance
      if (dist < linear_tolerance_ &&
          std::hypot(current_twist_.linear.x, current_twist_.linear.y) <
              linear_vel_tolerance_) {

        RCLCPP_INFO(get_logger(), "Segment %zu: MOVE done (dist=%.4f m).",
                    current_idx_, dist);

        // ── Anti-drift compensation ──────────────────────────────────
        // If the robot's heading drifted significantly during the move,
        // inject that drift into the NEXT waypoint's turn.
        // This keeps heading errors from accumulating across waypoints.
        const size_t next_idx = current_idx_ + 1;
        if (anti_drift_ && std::abs(yaw_drift) > yaw_drift_threshold_ &&
            next_idx < waypoints_.size()) {
          waypoints_[next_idx].dyaw += yaw_drift;
          RCLCPP_INFO(get_logger(),
                      "Anti-drift: yaw_drift=%.4f rad → added to wp %zu "
                      "(new dyaw=%.4f)",
                      yaw_drift, next_idx, waypoints_[next_idx].dyaw);
        }

        current_idx_++;
        segment_active_ = false;
        phase_ = Phase::TURN;
        stop();

        // Pause before next segment
        pausing_ = true;
        pause_ticks_ = 0;
        return;
      }

      compute_move_pid(ex_b, ey_b, dex, dey);

      // Wall avoidance modifies cmd_ before it is published
      apply_wall_avoidance();
    }

    cmd_vel_pub_->publish(cmd_);
  }

  // ── Segment activation ────────────────────────────────────────────────────

  void start_segment() {
    const auto &wp = waypoints_[current_idx_];

    // Step 1: target yaw = current yaw + this waypoint's yaw delta
    target_yaw_ = current_yaw_ + wp.dyaw;

    // Step 2: dx/dy are in the robot frame AFTER the turn.
    // Rotate them by target_yaw_ to get the world-frame displacement.
    double dx_world =
        wp.dx * std::cos(target_yaw_) - wp.dy * std::sin(target_yaw_);
    double dy_world =
        wp.dx * std::sin(target_yaw_) + wp.dy * std::cos(target_yaw_);

    target_x_ = current_x_ + dx_world;
    target_y_ = current_y_ + dy_world;

    phase_ = Phase::TURN;
    segment_active_ = true;

    // Reset all integrators at the start of every segment
    sum_I_x_ = sum_I_y_ = sum_I_yaw_ = 0.0;

    RCLCPP_INFO(get_logger(),
                "Segment %zu: wp(dx=%.3f dy=%.3f dyaw=%.3f) → "
                "target_yaw=%.3f target=(%.3f, %.3f)",
                current_idx_, wp.dx, wp.dy, wp.dyaw, target_yaw_, target_x_,
                target_y_);
  }

  // ── PID computations ──────────────────────────────────────────────────────

  // Turn PID — outputs angular velocity only
  // e_yaw  : heading error  (rad)
  // de_yaw : 0 - imu_yaw_rate  (derivative of heading error toward zero)
  void compute_turn_pid(double e_yaw, double de_yaw) {
    // Integrate error over time (dt_ = 1/20 = 0.05 s)
    sum_I_yaw_ += e_yaw * dt_;

    double u =
        pid_yaw_.kp * e_yaw + pid_yaw_.ki * sum_I_yaw_ + pid_yaw_.kd * de_yaw;

    u = std::clamp(u, -max_angular_vel_, max_angular_vel_);

    cmd_.linear.x = 0.0;
    cmd_.linear.y = 0.0;
    cmd_.angular.z = u;
  }

  // Move PID — outputs linear velocities only
  // ex   : robot-frame forward  error (metres)
  // ey   : robot-frame lateral  error (metres)
  // dex  : 0 - odom_vel_x   (D-term input for forward axis)
  // dey  : 0 - odom_vel_y   (D-term input for lateral axis)
  void compute_move_pid(double ex, double ey, double dex, double dey) {
    sum_I_x_ += ex * dt_;
    sum_I_y_ += ey * dt_;

    cmd_.linear.x = pid_x_.kp * ex + pid_x_.ki * sum_I_x_ + pid_x_.kd * dex;
    cmd_.linear.y = pid_y_.kp * ey + pid_y_.ki * sum_I_y_ + pid_y_.kd * dey;
    cmd_.angular.z = 0.0;

    // Scale down both axes together to preserve motion direction
    double vnorm = std::hypot(cmd_.linear.x, cmd_.linear.y);
    if (vnorm > max_linear_vel_) {
      cmd_.linear.x *= max_linear_vel_ / vnorm;
      cmd_.linear.y *= max_linear_vel_ / vnorm;
    }
  }

  // ── Wall avoidance ────────────────────────────────────────────────────────
  // Called after compute_move_pid — may reduce or zero cmd_ velocity.
  //
  // Key idea: instead of checking a fixed "front" direction, this checks
  // all beams within ±front_angle_rad_ of the CURRENT MOTION DIRECTION.
  // So if the robot is strafing, it checks the strafe direction — not just
  // straight ahead. This prevents wall collisions regardless of move type.
  void apply_wall_avoidance() {
    if (!latest_scan_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "No laser scan available for wall avoidance.");
      return;
    }

    const double vx = cmd_.linear.x;
    const double vy = cmd_.linear.y;
    const double v_norm = std::hypot(vx, vy);

    // Nothing to check if the robot is not commanding motion
    if (v_norm <= 1e-3)
      return;

    const auto &scan = *latest_scan_;
    if (scan.ranges.empty())
      return;

    // Compute the direction of motion in base_link frame (robot frame)
    const double motion_dir_base = std::atan2(vy, vx);

    float min_range = std::numeric_limits<float>::infinity();
    float min_angle = 0.0f;

    double beam_angle_scan = scan.angle_min;
    for (std::size_t i = 0; i < scan.ranges.size();
         ++i, beam_angle_scan += scan.angle_increment) {

      // Convert beam angle from laser frame → base_link frame.
      // laser_yaw_in_base_ = π for ROSBot XL (laser mounted backwards).
      double beam_angle_base =
          normalize_angle(beam_angle_scan + laser_yaw_in_base_);

      // Only process beams that are roughly in the direction we are moving
      double diff = normalize_angle(beam_angle_base - motion_dir_base);
      if (std::abs(diff) > front_angle_rad_)
        continue;

      const float r = scan.ranges[i];
      if (!std::isfinite(r))
        continue;

      // Skip antenna self-echo (ROSBot XL rear antenna reflects lidar)
      if (is_antenna_echo(beam_angle_base, static_cast<double>(r)))
        continue;

      if (r < min_range) {
        min_range = r;
        min_angle = static_cast<float>(beam_angle_base);
      }
    }

    if (!std::isfinite(min_range))
      return;

    // Hard stop: obstacle is closer than stop_distance in motion direction
    if (min_range < stop_distance_) {
      RCLCPP_WARN(get_logger(),
                  "[AVOID] Hard stop: obstacle at %.3f m, angle=%.2f rad "
                  "(motion dir=%.2f rad)",
                  min_range, min_angle, motion_dir_base);
      cmd_.linear.x = 0.0;
      cmd_.linear.y = 0.0;
      return;
    }

    // Soft slowdown: proportionally reduce speed as obstacle approaches
    if (min_range < slow_distance_) {
      // scale = 0 at stop_distance, scale = 1 at slow_distance
      double scale =
          (min_range - stop_distance_) / (slow_distance_ - stop_distance_);
      scale = std::clamp(scale, 0.0, 1.0);

      double v_cap = scale * max_linear_vel_;
      if (v_norm > v_cap) {
        double k = v_cap / v_norm;
        cmd_.linear.x *= k;
        cmd_.linear.y *= k;
      }

      RCLCPP_DEBUG(get_logger(),
                   "[AVOID] Slowing: obstacle %.3f m, scale=%.2f, v_cap=%.3f",
                   min_range, scale, v_cap);
    }
  }

  // ── Helpers ───────────────────────────────────────────────────────────────

  // Returns true if a beam at angle_base (base_link frame) with this range
  // is likely an antenna self-echo and should be ignored.
  bool is_antenna_echo(double angle_base, double range) const {
    double d = normalize_angle(angle_base - antenna_center_);
    return (std::abs(d) < antenna_half_width_) && (range < antenna_max_range_);
  }

  // Publish a zero Twist (stop all motion)
  void stop() {
    cmd_ = geometry_msgs::msg::Twist{};
    cmd_vel_pub_->publish(cmd_);
  }

  // Wrap angle to [-π, π]
  static double normalize_angle(double theta) {
    while (theta > M_PI)
      theta -= 2.0 * M_PI;
    while (theta < -M_PI)
      theta += 2.0 * M_PI;
    return theta;
  }
};

// ═══════════════════════════════════════════════════════════════════════════
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Scene number selects: waypoint file + config file
  //   1, 3, 5 → simulation   → sim_params.yaml
  //   2, 4, 6 → real robot   → real_params.yaml
  int scene_number = 1;
  if (argc > 1)
    scene_number = std::atoi(argv[1]);

  // Select the right parameter file based on scene
  std::string config_file;
  if (scene_number == 1 || scene_number == 3 || scene_number == 5) {
    config_file = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/pid_maze_solver/"
                  "config/sim_2.yaml";
  } else if (scene_number == 2 || scene_number == 4 || scene_number == 6) {
    config_file = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/pid_maze_solver/"
                  "config/real_2.yaml";
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("maze_solver_node"),
                 "Invalid scene number: %d. Must be 1–6.", scene_number);
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(rclcpp::get_logger("maze_solver_node"),
              "Scene %d → loading config: %s", scene_number,
              config_file.c_str());

  // Pass the config file path as a ROS argument so declare_parameter()
  // picks up values from it automatically.
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", config_file});

  auto node = std::make_shared<PIDMazeSolver>(scene_number, options);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    4);
  executor.add_node(node);

  try {
    executor.spin();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}