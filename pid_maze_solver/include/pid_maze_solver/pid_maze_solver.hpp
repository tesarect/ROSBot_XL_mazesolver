#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pid_maze_solver/pid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cstddef>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

using Twist = geometry_msgs::msg::Twist;
using Odom  = nav_msgs::msg::Odometry;
using Laser = sensor_msgs::msg::LaserScan;

class PIDMazeSolver : public rclcpp::Node {
public:
  PIDMazeSolver(int scene_number,
                const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  // ── Waypoint: relative delta {dx, dy, dyaw} ───────────────────────────────
  // x/y = metres to move from current pose at activation
  // yaw = radians to turn from current yaw at activation
  struct WayPoint {
    float x{0.0f};
    float y{0.0f};
    float yaw{0.0f};
  };

  // ── Current pose from odom ────────────────────────────────────────────────
  struct PoseOrient {
    float x{0.0f};
    float y{0.0f};
    float yaw{0.0f};
  };

  // ── State machine ─────────────────────────────────────────────────────────
  enum class State {
    INITIAL,    // wait for odom + laser, load waypoints
    NEXT,       // check if at waypoint, decide next state
    TURN,       // rotate by yaw delta
    MOVE,       // translate by x/y delta
    CORRECTING, // fine-adjust position using laser before moving on
    FINAL       // stop and shutdown
  };

  struct Opening {
    double center_angle_deg;
    double width_deg;
    double avg_range;
  };

private:
  struct LaserDirections {
    int front_index;
    int back_index;
    int left_index;
    int right_index;
  };

  // ── PID ───────────────────────────────────────────────────────────────────
  float TkP_, TkI_, TkD_;
  float DkP_, DkI_, DkD_;
  maze_solver::PID distance_pid_;
  maze_solver::PID turn_pid_;
  float max_lin_vel_;
  float max_ang_vel_;
  float yaw_tolerance_;
  float goal_tolerance_;

  // ── Laser thresholds (declare_parameter) ──────────────────────────────────
  float front_stop_thresh_;   // stop forward motion if dN < this
  float wall_nudge_thresh_;   // apply side nudge if dE or dW < this
  float nudge_gain_;          // velocity magnitude of side nudge
  float front_corr_thresh_;   // back up in CORRECTING if dN < this
  float side_corr_thresh_;    // push sideways in CORRECTING if dE/dW < this
  float tilt_thresh_;         // min diagonal diff to apply tilt correction
  float tilt_gain_;           // angular gain for tilt correction
  int   correction_timeout_;  // max CORRECTING cycles before moving on

  // ── Topics / frames ───────────────────────────────────────────────────────
  std::string odom_topic_;
  std::string laser_topic_;
  std::string base_link_;
  std::string laser_link_;

  // ── Odom state ────────────────────────────────────────────────────────────
  bool        initial_odom_received_{false};
  PoseOrient  current_pose_;

  // ── Goal anchoring (set at activation of each waypoint) ───────────────────
  float start_x_{0.0f};
  float start_y_{0.0f};
  float start_yaw_{0.0f};

  // ── Waypoints ─────────────────────────────────────────────────────────────
  std::vector<WayPoint> waypoints_;   // loaded from YAML
  size_t  current_goal_idx_{0};
  bool    goal_active_{false};
  int     correction_counter_{0};
  int     scene_number_;

  // ── Laser ─────────────────────────────────────────────────────────────────
  bool    initial_laser_received_{false};
  bool    laser_initialized_{false};
  double  laser_to_base_yaw_{0.0};
  double  laser_offset_deg_;
  int     front_idx_;
  int     north_idx_, south_idx_, east_idx_, west_idx_;
  int     ne_idx_,   nw_idx_,    se_idx_,   sw_idx_;
  double  dN{0}, dS{0}, dE{0}, dW{0};
  double  dNE{0}, dNW{0}, dSE{0}, dSW{0};
  int     band_half_;
  double  angle_increment_;
  double  angle_min_;
  int     num_rays_;

  double BAND_DEGREES      = 10.0;
  double OPENING_MIN_RANGE = 0.50;
  double OPENING_MIN_DEG   = 20.0;
  double MAX_RANGE_CLIP    = 3.50;

  std::vector<Opening> openings_;

  // ── TF ────────────────────────────────────────────────────────────────────
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── ROS interfaces ────────────────────────────────────────────────────────
  rclcpp::Publisher<Twist>::SharedPtr    twist_pub_;
  rclcpp::Subscription<Odom>::SharedPtr  odom_sub_;
  rclcpp::Subscription<Laser>::SharedPtr laser_sub_;
  rclcpp::SubscriptionOptions odom_sub_options_;
  rclcpp::SubscriptionOptions laser_sub_options_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ── State machine ─────────────────────────────────────────────────────────
  State state_{State::INITIAL};

  // ── Core state functions ──────────────────────────────────────────────────
  void timer_callback();
  void get_next_wp();
  void face_next_wp();
  void head_to_next_wp();
  void do_correction();

  // ── Alignment utility (not called — for manual testing) ───────────────────
  // Rotates robot so base_link X axis is parallel to odom X axis.
  // same_direction=true  → face yaw=0   (+X, same as odom)
  // same_direction=false → face yaw=π   (-X, opposite to odom)
  void align_to_odom_x(bool same_direction);

  // ── Laser helpers ─────────────────────────────────────────────────────────
  void   initLaser(const Laser::SharedPtr msg);
  void   laserCallback(const Laser::SharedPtr msg);
  static double bandAvg(const std::vector<float> &ranges, int center,
                        int half, double clip, float range_max);
  LaserDirections compute_laser_indices(const Laser &scan);

  // ── Parameter / YAML loading ──────────────────────────────────────────────
  void LoadParameters();
  void LoadWaypointsYaml();

  // ── Utilities ─────────────────────────────────────────────────────────────
  void  publish_vel(double vx, double vy, double wz) const;
  void  StopRobot();
  bool  isWithinGoalTolerance() const;
  static float NormalizeAngle(float angle);
  std::string  prnt_waypoints() const;
};