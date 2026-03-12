#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pid_maze_solver/pid.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

// delta distance + odom correction + parallel wall correction at goal (using
// laser) + heading hold

class PIDMazeSolver : public rclcpp::Node {
public:
  PIDMazeSolver(int scene_number, const rclcpp::NodeOptions &options)
      : Node("maze_solver_node", options), scene_number_(scene_number),
        tf_buffer_(this->get_clock()) {

    // ── Parameters ─────────────────────────────────────────────────────────
    odom_topic_ = this->declare_parameter<std::string>("odom_topic",
                                                       "/odometry/filtered");
    laser_topic_ = this->declare_parameter<std::string>("laser_topic", "/scan");
    base_link_ = this->declare_parameter<std::string>("base_link", "base_link");

    TkP_ = this->declare_parameter<float>("TkP", 1.2f);
    TkI_ = this->declare_parameter<float>("TkI", 0.001f);
    TkD_ = this->declare_parameter<float>("TkD", 0.001f);
    DkP_ = this->declare_parameter<float>("DkP", 1.2f);
    DkI_ = this->declare_parameter<float>("DkI", 0.001f);
    DkD_ = this->declare_parameter<float>("DkD", 0.001f);

    max_lin_vel_ = this->declare_parameter<float>("max_lin_vel", 0.2f);
    max_ang_vel_ = this->declare_parameter<float>("max_ang_vel", 1.0f);
    goal_tolerance_ = this->declare_parameter<float>("goal_tolerance", 0.02f);
    yaw_tolerance_ = this->declare_parameter<float>("yaw_tolerance", 0.05f);

    front_stop_thresh_ =
        this->declare_parameter<float>("front_stop_thresh", 0.22f);
    wall_nudge_thresh_ =
        this->declare_parameter<float>("wall_nudge_thresh", 0.20f);
    nudge_gain_ = this->declare_parameter<float>("nudge_gain", 0.05f);
    front_corr_thresh_ =
        this->declare_parameter<float>("front_corr_thresh", 0.16f);
    early_stop_dist_ = this->declare_parameter<float>("early_stop_dist", 0.15f);
    side_corr_thresh_ =
        this->declare_parameter<float>("side_corr_thresh", 0.20f);
    tilt_correction_ = this->declare_parameter<bool>("tilt_correction", false);
    tilt_thresh_ = this->declare_parameter<float>("tilt_thresh", 0.015f);
    tilt_gain_ = this->declare_parameter<float>("tilt_gain", 1.0f);
    correction_timeout_ =
        this->declare_parameter<int>("correction_timeout", 200);

    parallel_thresh_ = this->declare_parameter<float>("parallel_thresh", 0.02f);
    parallel_wall_max_ =
        this->declare_parameter<float>("parallel_wall_max", 0.60f);
    parallel_timeout_ = this->declare_parameter<int>("parallel_timeout", 50);
    parallel_gain_ = this->declare_parameter<float>("parallel_gain", 0.8f);
    corner_thresh_ = this->declare_parameter<float>("corner_thresh", 0.12f);

    // ── Heading hold during MOVE ───────────────────────────────────────────
    // Adds a wz correction during transit to hold heading at
    // start_yaw_+goal.theta
    heading_hold_kP_ = this->declare_parameter<float>("heading_hold_kP", 1.5f);
    heading_hold_max_wz_ =
        this->declare_parameter<float>("heading_hold_max_wz", 0.3f);

    // ── PID setup ──────────────────────────────────────────────────────────
    RCLCPP_INFO(
        this->get_logger(),
        " Dist PID gains: DkP=%.3f DkI=%.3f DkD=%.3f | max_lin_vel=%.3f", DkP_,
        DkI_, DkD_, max_lin_vel_);
    RCLCPP_INFO(
        this->get_logger(),
        " Turn PID gains: TkP=%.3f TkI=%.3f TkD=%.3f | max_ang_vel=%.3f", TkP_,
        TkI_, TkD_, max_ang_vel_);

    turn_pid_.set_gain(TkP_, TkI_, TkD_);
    turn_pid_.set_limit(max_ang_vel_, 1.0f);
    dist_pid_.set_gain(DkP_, DkI_, DkD_);
    dist_pid_.set_limit(max_lin_vel_, 1.0f);

    // ── TF ─────────────────────────────────────────────────────────────────
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this);

    // ── Load waypoints ─────────────────────────────────────────────────────
    loadWaypoints();

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    // ── Subscribers ────────────────────────────────────────────────────────
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          current_x_ = msg->pose.pose.position.x;
          current_y_ = msg->pose.pose.position.y;
          float qz = msg->pose.pose.orientation.z;
          float qw = msg->pose.pose.orientation.w;
          current_yaw_ = 2.0f * std::atan2(qz, qw);
          if (!initialized_) {
            RCLCPP_INFO(get_logger(), "Odom ready: (%.3f, %.3f) yaw=%.3f",
                        current_x_, current_y_, current_yaw_);
            initialized_ = true;
          }
        });

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        laser_topic_, 10,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          if (!laser_init_)
            initLaser(msg);
          else
            updateLaser(msg);
        });

    // ── Control loop timer at 10 Hz ────────────────────────────────────────
    timer_ = this->create_wall_timer(
        100ms, std::bind(&PIDMazeSolver::controlLoop, this));

    RCLCPP_INFO(get_logger(), "PIDMazeSolver ready. Scene=%d", scene_number_);
  }

private:
  // ── Types ──────────────────────────────────────────────────────────────────
  struct Goal {
    float x;
    float y;
    float theta;
  };
  enum class Phase { TURNING, MOVING, CORRECTING };

  // ── Scene / waypoints ──────────────────────────────────────────────────────
  int scene_number_;
  std::vector<Goal> goals_;
  size_t goal_idx_ = 0;
  bool goal_active_ = false;
  Phase phase_ = Phase::TURNING;

  // ── Odom ───────────────────────────────────────────────────────────────────
  bool initialized_ = false;
  float current_x_ = 0.0f, current_y_ = 0.0f, current_yaw_ = 0.0f;
  float start_x_ = 0.0f, start_y_ = 0.0f, start_yaw_ = 0.0f;

  // ── PID ────────────────────────────────────────────────────────────────────
  maze_solver::PID turn_pid_;
  maze_solver::PID dist_pid_;
  float TkP_, TkI_, TkD_;
  float DkP_, DkI_, DkD_;
  float max_lin_vel_, max_ang_vel_;
  float goal_tolerance_, yaw_tolerance_;

  // Manual PID state for XY (separate from pid.hpp — needed for X and Y axes)
  float prev_err_x_ = 0.0f, integ_x_ = 0.0f;
  float prev_err_y_ = 0.0f, integ_y_ = 0.0f;

  // ── Laser thresholds ───────────────────────────────────────────────────────
  float front_stop_thresh_;
  float early_stop_dist_;
  float wall_nudge_thresh_, nudge_gain_;
  float front_corr_thresh_, side_corr_thresh_;
  bool tilt_correction_;
  float tilt_thresh_, tilt_gain_;
  int correction_timeout_, correction_counter_ = 0;

  // ── Laser readings ─────────────────────────────────────────────────────────
  float front_ = 9.9f, back_ = 9.9f;
  float left_ = 9.9f, right_ = 9.9f;
  // Diagonal halves for tilt detection (near/far from front)
  float left_front_avg_ = 9.9f, left_back_avg_ = 9.9f;
  float right_front_avg_ = 9.9f, right_back_avg_ = 9.9f;
  // Wall-parallel detection: NEE=67.5°, SEE=112.5°, NWW=-67.5°, SWW=-112.5°
  // These avoid corners (pure E/W) and open bays (pure N/S diagonals)
  float nee_ = 9.9f, see_ = 9.9f; // right-side pair
  float nww_ = 9.9f, sww_ = 9.9f; // left-side pair

  // ── Laser init ─────────────────────────────────────────────────────────────
  bool laser_init_ = false;
  double laser_yaw_offset_ = 0.0;
  float angle_min_ = 0.0f;
  float angle_inc_ = 0.0f;
  int num_rays_ = 0;
  int idx_front_, idx_back_, idx_left_, idx_right_;
  int idx_lf_, idx_lb_, idx_rf_, idx_rb_;     // tilt band edges
  int idx_nee_, idx_see_, idx_nww_, idx_sww_; // wall-parallel indices
  int band_half_ = 5;

  // ── Wall-parallel alignment params ────────────────────────────────────────
  float parallel_thresh_;   // max |NEE-SEE| or |NWW-SWW| to consider parallel
  float parallel_wall_max_; // ignore walls farther than this (open bay)
  int parallel_timeout_;    // max cycles before giving up
  float parallel_gain_;     // fixed angular gain for parallel correction
  float corner_thresh_;     // max deviation of E from straight-wall prediction

  // ── Heading hold during MOVE ───────────────────────────────────────────────
  float heading_hold_kP_;     // P gain for in-transit heading correction
  float heading_hold_max_wz_; // max wz correction during MOVE

  // ── Topics / frames ────────────────────────────────────────────────────────
  std::string odom_topic_, laser_topic_, base_link_;

  // ── TF ─────────────────────────────────────────────────────────────────────
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── ROS ────────────────────────────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ═══════════════════════════════════════════════════════════════════════════
  // loadWaypoints
  // ═══════════════════════════════════════════════════════════════════════════
  void loadWaypoints() {
    std::string path;
    switch (scene_number_) {
    case 1:
      path = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/resources"
             "/waypoints/maze_waypoints_sim.yaml";
      break;
    case 2:
      path = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/resources"
             "/waypoints/maze_waypoints_real.yaml";
      break;
    case 3:
      path = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/resources"
             "/waypoints/rev_maze_waypoints_sim.yaml";
      break;
    case 4:
      path = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/resources"
             "/waypoints/rev_maze_waypoints_real.yaml";
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Invalid scene number: %d", scene_number_);
      return;
    }

    RCLCPP_INFO(get_logger(), "Loading waypoints: %s", path.c_str());
    YAML::Node cfg;
    try {
      cfg = YAML::LoadFile(path);
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(get_logger(), "YAML error: %s", e.what());
      return;
    }

    for (size_t i = 0; i < cfg["waypoints"].size(); i++) {
      const auto &wp = cfg["waypoints"][i];
      goals_.push_back(
          {wp[0].as<float>(), wp[1].as<float>(), wp[2].as<float>()});
      RCLCPP_INFO(get_logger(), "  WP[%zu]: x=%.3f y=%.3f theta=%.3f", i,
                  goals_.back().x, goals_.back().y, goals_.back().theta);
    }
    RCLCPP_INFO(get_logger(), "%zu waypoints loaded.", goals_.size());
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // initLaser — called once on first scan
  // Uses TF to find laser mount yaw, computes direction indices correctly
  // for any laser mounting (sim 180° flip or real robot)
  // ═══════════════════════════════════════════════════════════════════════════
  void initLaser(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(base_link_, msg->header.frame_id,
                                      msg->header.stamp,
                                      rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "TF not ready: %s", ex.what());
      return;
    }

    double roll, pitch, yaw;
    tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                      tf.transform.rotation.z, tf.transform.rotation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    laser_yaw_offset_ = yaw;

    angle_min_ = msg->angle_min;
    angle_inc_ = msg->angle_increment;
    num_rays_ = static_cast<int>(msg->ranges.size());
    band_half_ = std::max(1, static_cast<int>(std::round((10.0 * M_PI / 180.0) /
                                                         angle_inc_))); // ±10°

    auto toIdx = [&](double base_deg) {
      double laser_rad = base_deg * M_PI / 180.0 - laser_yaw_offset_;
      while (laser_rad > M_PI)
        laser_rad -= 2 * M_PI;
      while (laser_rad < -M_PI)
        laser_rad += 2 * M_PI;
      int idx =
          static_cast<int>(std::round((laser_rad - angle_min_) / angle_inc_));
      return std::clamp(idx, 0, num_rays_ - 1);
    };

    idx_front_ = toIdx(0.0);
    idx_back_ = toIdx(180.0);
    idx_left_ = toIdx(90.0);   // +Y = left in ROS
    idx_right_ = toIdx(-90.0); // -Y = right in ROS

    // Diagonal band edges for tilt detection
    // left_front = 45–90°, left_back = 90–135°
    // right_front = -45 to -90°, right_back = -90 to -135°
    idx_lf_ = toIdx(45.0);
    idx_lb_ = toIdx(135.0);
    idx_rf_ = toIdx(-45.0);
    idx_rb_ = toIdx(-135.0);

    // Wall-parallel indices — midway between diagonal and cardinal
    // Avoids pure E/W (corners) and pure N/S diagonals (open bays)
    // Right side: NEE=67.5°, SEE=112.5°
    // Left side:  NWW=-67.5°, SWW=-112.5°
    idx_nee_ = toIdx(67.5);
    idx_see_ = toIdx(112.5);
    idx_nww_ = toIdx(-67.5);
    idx_sww_ = toIdx(-112.5);

    laser_init_ = true;
    RCLCPP_INFO(get_logger(),
                "Laser init done | yaw_offset=%.4f rad | band=±%d rays",
                laser_yaw_offset_, band_half_);
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // updateLaser — called every scan, updates direction readings
  // ═══════════════════════════════════════════════════════════════════════════
  void updateLaser(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const auto &r = msg->ranges;
    float rmax = msg->range_max;

    auto avg = [&](int center) {
      float sum = 0.0f;
      int cnt = 0;
      for (int i = std::max(0, center - band_half_);
           i <= std::min(num_rays_ - 1, center + band_half_); i++) {
        if (std::isfinite(r[i]) && r[i] > 0.01f && r[i] < rmax)
          sum += r[i], cnt++;
      }
      return cnt > 0 ? sum / cnt : 9.9f;
    };

    auto avgRange = [&](int a, int b) {
      // average over a range of indices (handles reversed order)
      int lo = std::min(a, b), hi = std::max(a, b);
      float sum = 0.0f;
      int cnt = 0;
      for (int i = std::max(0, lo); i <= std::min(num_rays_ - 1, hi); i++) {
        if (std::isfinite(r[i]) && r[i] > 0.01f && r[i] < rmax)
          sum += r[i], cnt++;
      }
      return cnt > 0 ? sum / cnt : 9.9f;
    };

    front_ = avg(idx_front_);
    back_ = avg(idx_back_);
    left_ = avg(idx_left_);
    right_ = avg(idx_right_);

    // Tilt detection: split left/right wall into front-half and back-half
    left_front_avg_ = avgRange(idx_lf_, idx_left_);
    left_back_avg_ = avgRange(idx_left_, idx_lb_);
    right_front_avg_ = avgRange(idx_rf_, idx_right_);
    right_back_avg_ = avgRange(idx_right_, idx_rb_);

    // Wall-parallel readings
    nee_ = avg(idx_nee_); // right-side front
    see_ = avg(idx_see_); // right-side back
    nww_ = avg(idx_nww_); // left-side front
    sww_ = avg(idx_sww_); // left-side back
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // controlLoop — timer callback, runs every 100ms
  // Phase: TURNING → MOVING → CORRECTING → next goal
  // ═══════════════════════════════════════════════════════════════════════════
  void controlLoop() {
    if (!initialized_ || !laser_init_)
      return;

    // All done
    if (goal_idx_ >= goals_.size()) {
      publish(0, 0, 0);
      timer_->cancel();
      RCLCPP_INFO(get_logger(), "All waypoints complete. Done!");
      rclcpp::shutdown();
      return;
    }

    const Goal &goal = goals_[goal_idx_];

    // ── Activate new goal ──────────────────────────────────────────────────
    if (!goal_active_) {
      start_x_ = current_x_;
      start_y_ = current_y_;
      start_yaw_ = current_yaw_;
      goal_active_ = true;
      phase_ = Phase::TURNING;
      turn_pid_.reset();
      dist_pid_.reset();
      integ_x_ = 0;
      integ_y_ = 0;
      prev_err_x_ = 0;
      prev_err_y_ = 0;
      RCLCPP_INFO(get_logger(),
                  "Goal %zu | turn=%.3f rad | move dx=%.3f dy=%.3f", goal_idx_,
                  goal.theta, goal.x, goal.y);
    }

    // ── TURNING ───────────────────────────────────────────────────────────
    if (phase_ == Phase::TURNING) {
      // Skip turn if no yaw delta
      if (std::fabs(goal.theta) < 0.01f) {
        phase_ = Phase::MOVING;
        return;
      }

      float target_yaw = normalizeAngle(start_yaw_ + goal.theta);
      float err_yaw = normalizeAngle(target_yaw - current_yaw_);

      if (std::fabs(err_yaw) < yaw_tolerance_) {
        publish(0, 0, 0);
        turn_pid_.reset();
        RCLCPP_INFO(get_logger(), "[TURN] Done | err=%.4f", err_yaw);
        phase_ = Phase::MOVING;
        return;
      }

      // Use dt=0.1 (100ms timer interval) — matches his approach
      double ang_vel = turn_pid_.compute(err_yaw, 0.1);
      publish(0, 0, ang_vel);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                           "[TURN] err=%.4f rad | ang_vel=%.3f", err_yaw,
                           ang_vel);
      return;
    }

    // ── MOVING ────────────────────────────────────────────────────────────
    if (phase_ == Phase::MOVING) {
      // Skip move if no xy delta
      if (std::fabs(goal.x) < 0.001f && std::fabs(goal.y) < 0.001f) {
        phase_ = Phase::CORRECTING;
        correction_counter_ = 0;
        return;
      }

      float target_x = start_x_ + goal.x;
      float target_y = start_y_ + goal.y;
      float err_x = target_x - current_x_;
      float err_y = target_y - current_y_;
      float dist = std::sqrt(err_x * err_x + err_y * err_y);

      // ── Front obstacle early stop ──────────────────────────────────────
      // STOP EARLY WHEN WALL DETECTED, AND CONSIDER THAT AS GOAL REACHED
      if (front_ <= front_stop_thresh_) {
        if (dist < early_stop_dist_) {
          // close enough — accept wall as goal reached
          publish(0, 0, 0);
          rclcpp::sleep_for(200ms);
          RCLCPP_WARN(get_logger(),
                      "[MOVE] Front obstacle %.3f — dist=%.3f < "
                      "early_stop_dist=%.3f, accepting",
                      front_, dist, early_stop_dist_);
          phase_ = Phase::CORRECTING;
          correction_counter_ = 0;
        } else {
          // too far — something wrong, just advance anyway after logging
          publish(0, 0, 0);
          RCLCPP_WARN(get_logger(),
                      "[MOVE] Front obstacle %.3f — dist=%.3f too large "
                      "[early_stop: %.3f] | forcing advance",
                      front_, dist, early_stop_dist_);
          phase_ = Phase::CORRECTING;
          correction_counter_ = 0;
        }
        return;
      }

      // ── Goal reached ──────────────────────────────────────────────────
      if (dist < goal_tolerance_) {
        publish(0, 0, 0);
        rclcpp::sleep_for(300ms);
        RCLCPP_INFO(get_logger(), "[MOVE] Reached goal %zu", goal_idx_);
        phase_ = Phase::CORRECTING;
        correction_counter_ = 0;
        return;
      }

      // ── PID: compute world-frame velocity then rotate to robot frame ───
      integ_x_ += err_x * 0.1f;
      integ_y_ += err_y * 0.1f;
      float ctrl_x =
          DkP_ * err_x + DkI_ * integ_x_ + DkD_ * (err_x - prev_err_x_) / 0.1f;
      float ctrl_y =
          DkP_ * err_y + DkI_ * integ_y_ + DkD_ * (err_y - prev_err_y_) / 0.1f;
      ctrl_x = std::clamp(ctrl_x, -max_lin_vel_, max_lin_vel_);
      ctrl_y = std::clamp(ctrl_y, -max_lin_vel_, max_lin_vel_);

      // Rotate world-frame {ctrl_x, ctrl_y} → robot-frame {vx, vy}
      float cos_y = std::cos(current_yaw_);
      float sin_y = std::sin(current_yaw_);
      float vx = cos_y * ctrl_x + sin_y * ctrl_y;
      float vy = -sin_y * ctrl_x + cos_y * ctrl_y;

      // ── Wall nudge ────────────────────────────────────────────────────
      if (left_ < wall_nudge_thresh_) {
        vy -= nudge_gain_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "[MOVE] Left nudge left_=%.3f", left_);
      }
      if (right_ < wall_nudge_thresh_) {
        vy += nudge_gain_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "[MOVE] Right nudge right_=%.3f", right_);
      }

      // ── Heading hold ──────────────────────────────────────────────────
      // Hold heading at the target yaw (start_yaw_ + goal.theta) during MOVE.
      // Counters yaw drift without waiting for the next goal boundary.
      float target_yaw = normalizeAngle(start_yaw_ + goal.theta);
      float yaw_err = normalizeAngle(target_yaw - current_yaw_);
      float wz_hold = heading_hold_kP_ * yaw_err;
      wz_hold =
          std::clamp(wz_hold, -heading_hold_max_wz_, heading_hold_max_wz_);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                           "[MOVE] heading_hold yaw_err=%.4f wz=%.3f", yaw_err,
                           wz_hold);

      publish(vx, vy, wz_hold);

      prev_err_x_ = err_x;
      prev_err_y_ = err_y;

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                           "[MOVE] dist=%.4f | vx=%.3f vy=%.3f | "
                           "front=%.2f left=%.2f right=%.2f",
                           dist, vx, vy, front_, left_, right_);
      return;
    }

    // ── CORRECTING ────────────────────────────────────────────────────────
    if (phase_ == Phase::CORRECTING) {
      geometry_msgs::msg::Twist cmd;
      bool need = false;

      // Back up if too close to front wall
      if (front_ < front_corr_thresh_) {
        cmd.linear.x -= 0.03f;
        need = true;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "[CORR] Front too close: %.3f", front_);
      }

      // Push away from side walls
      if (left_ < side_corr_thresh_) {
        cmd.linear.y -= 0.03f; // left wall → push right
        need = true;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "[CORR] Left wall: %.3f", left_);
      }
      if (right_ < side_corr_thresh_) {
        cmd.linear.y += 0.03f; // right wall → push left
        need = true;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "[CORR] Right wall: %.3f", right_);
      }

      // ── Tilt correction using closer wall diagonal split ──────────────
      // Compare front-half vs back-half of the closer wall.
      // If front-half > back-half → nose pointing away from wall → rotate in
      // If front-half < back-half → nose pointing into wall  → rotate out
      if (tilt_correction_) {
        if (left_ < right_) {
          float diff = left_front_avg_ - left_back_avg_;
          if (std::isfinite(diff) && std::fabs(diff) > tilt_thresh_) {
            float corr = std::clamp(-tilt_gain_ * diff, -0.1f, 0.1f);
            cmd.angular.z += -corr;
            need = true;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                                 "[CORR] Left tilt diff=%.3f wz=%.3f", diff,
                                 corr);
          }
        } else {
          float diff = right_front_avg_ - right_back_avg_;
          if (std::isfinite(diff) && std::fabs(diff) > tilt_thresh_) {
            float corr = std::clamp(tilt_gain_ * diff, -0.1f, 0.1f);
            cmd.angular.z += corr;
            need = true;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                                 "[CORR] Right tilt diff=%.3f wz=%.3f", diff,
                                 corr);
          }
        }
      }

      if (need) {
        vel_pub_->publish(cmd);
        correction_counter_++;
        if (correction_counter_ > correction_timeout_) {
          RCLCPP_WARN(get_logger(), "[CORR] Timeout — moving on.");
          advanceGoal();
        }
        return;
      }

      // All clear — align to wall before advancing
      publish(0, 0, 0);
      rclcpp::sleep_for(300ms);
      RCLCPP_INFO(get_logger(), "[CORR] Done in %d cycles.",
                  correction_counter_);
      align_parallel_to_wall(); // ← comment out to disable
      advanceGoal();
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // align_parallel_to_wall
  // Called after any motion (MOVE or TURN) completes, before CORRECTING.
  // To disable: comment out the call site, not this function.
  //
  // Logic:
  //   1. Pick closer wall using NEE+SEE (right) vs NWW+SWW (left) averages
  //   2. Ignore if both walls are beyond parallel_wall_max_ (open bay)
  //   3. Compute tilt = front-angle - back-angle for chosen wall
  //      right wall: tilt = nee_ - see_
  //        tilt > 0 → nose pointing away from right wall → rotate CW  (wz < 0)
  //        tilt < 0 → nose pointing into  right wall → rotate CCW (wz > 0)
  //      left wall: tilt = nww_ - sww_
  //        tilt > 0 → nose pointing away from left wall → rotate CCW (wz > 0)
  //        tilt < 0 → nose pointing into  left wall → rotate CW  (wz < 0)
  //   4. Apply fixed-gain angular velocity until |tilt| < parallel_thresh_
  //      or parallel_timeout_ cycles elapsed
  // ═══════════════════════════════════════════════════════════════════════════
  void align_parallel_to_wall() {
    // nee_(67.5°)/see_(112.5°) are on the LEFT side  (positive Y in ROS)
    // nww_(-67.5°)/sww_(-112.5°) are on the RIGHT side (negative Y in ROS)
    float avg_left = (nee_ + see_) / 2.0f;
    float avg_right = (nww_ + sww_) / 2.0f;

    // Both walls too far — open bay, nothing to align to
    if (avg_right > parallel_wall_max_ && avg_left > parallel_wall_max_) {
      RCLCPP_INFO(get_logger(),
                  "[PARALLEL] Both walls open (R=%.2f L=%.2f > %.2f) — skip",
                  avg_right, avg_left, parallel_wall_max_);
      return;
    }

    // ── Symmetry check — reject corners ───────────────────────────────────
    // Straight wall → front_ray ≈ back_ray (both hit the same flat surface)
    // Corner        → front_ray ≠ back_ray (rays hit different surfaces)
    auto isStraightWall = [&](float front_ray, float back_ray) -> bool {
      return std::fabs(front_ray - back_ray) < corner_thresh_;
    };

    bool left_straight = isStraightWall(nee_, see_);
    bool right_straight = isStraightWall(nww_, sww_);

    RCLCPP_INFO(
        get_logger(),
        "[PARALLEL] R_avg=%.3f L_avg=%.3f | "
        "straight R=%d L=%d | "
        "diff R=%.3f(nww=%.3f sww=%.3f) L=%.3f(nee=%.3f see=%.3f) thresh=%.3f",
        avg_right, avg_left, right_straight, left_straight,
        std::fabs(nww_ - sww_), nww_, sww_, std::fabs(nee_ - see_), nee_, see_,
        corner_thresh_);

    bool use_right = false;
    if (right_straight && left_straight)
      use_right = (avg_right <= avg_left);
    else if (right_straight)
      use_right = true;
    else if (left_straight)
      use_right = false;
    else {
      RCLCPP_WARN(get_logger(),
                  "[PARALLEL] Both walls are corners — skip alignment");
      return;
    }

    RCLCPP_INFO(get_logger(), "[PARALLEL] Aligning to %s wall",
                use_right ? "RIGHT" : "LEFT");

    rclcpp::Rate rate(10);
    int cycles = 0;

    while (rclcpp::ok() && cycles < parallel_timeout_) {
      float tilt, wz;

      if (use_right) {
        tilt = nww_ - sww_;
        // tilt > 0: nose away from right wall → rotate CW  → wz negative
        // tilt < 0: nose into  right wall     → rotate CCW → wz positive
        wz = -parallel_gain_ * tilt;
      } else {
        tilt = nee_ - see_;
        // tilt > 0: nose away from left wall → rotate CCW → wz positive
        // tilt < 0: nose into  left wall     → rotate CW  → wz negative
        wz = parallel_gain_ * tilt;
      }

      if (std::fabs(tilt) < parallel_thresh_) {
        publish(0, 0, 0);
        RCLCPP_INFO(get_logger(),
                    "[PARALLEL] Done | tilt=%.4f < thresh=%.4f | %d cycles",
                    tilt, parallel_thresh_, cycles);
        return;
      }

      wz = std::clamp(wz, -max_ang_vel_, max_ang_vel_);
      publish(0, 0, wz);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
                           "[PARALLEL] tilt=%.4f wz=%.3f | "
                           "nww=%.3f sww=%.3f nee=%.3f see=%.3f",
                           tilt, wz, nww_, sww_, nee_, see_);
      rate.sleep();
      cycles++;
    }

    publish(0, 0, 0);
    RCLCPP_WARN(get_logger(),
                "[PARALLEL] Timeout after %d cycles — continuing anyway",
                cycles);
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // advanceGoal — move to next waypoint, reset state
  // ═══════════════════════════════════════════════════════════════════════════
  void advanceGoal() {
    goal_idx_++;
    goal_active_ = false;
    correction_counter_ = 0;
    integ_x_ = 0;
    integ_y_ = 0;
    prev_err_x_ = 0;
    prev_err_y_ = 0;
    RCLCPP_INFO(get_logger(), "Advancing to goal %zu", goal_idx_);
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Utilities
  // ═══════════════════════════════════════════════════════════════════════════
  void publish(double vx, double vy, double wz) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.angular.z = wz;
    vel_pub_->publish(cmd);
  }

  float normalizeAngle(float a) {
    while (a > M_PI)
      a -= 2.0f * M_PI;
    while (a < -M_PI)
      a += 2.0f * M_PI;
    return a;
  }
};

// ═══════════════════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  int scene_number = 1;
  if (argc > 1) {
    scene_number = std::atoi(argv[1]);
  }

  std::string config_file;

  if (scene_number == 1 || scene_number == 3 || scene_number == 5) {
    config_file = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/pid_maze_solver/"
                  "config/sim.yaml";
  } else if (scene_number == 2 || scene_number == 4 || scene_number == 6) {
    config_file = "/home/user/ros2_ws/src/ROSBot_XL_mazesolver/pid_maze_solver/"
                  "config/real.yaml";
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("maze_solver_node"),
                 "Invalid scene number: %d. Must be 1–6.", scene_number);

    return 1;
  }
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", config_file});
  auto node = std::make_shared<PIDMazeSolver>(scene_number, options);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}