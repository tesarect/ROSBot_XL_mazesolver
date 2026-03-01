#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <Eigen/Dense>
#include <ostream>
#include <vector>

// Aliases for convenience
using Twist = geometry_msgs::msg::Twist;
using Odom = nav_msgs::msg::Odometry;
using Laser = sensor_msgs::msg::LaserScan;

class PIDMazeSolver : public rclcpp::Node {
public:
  //   PIDMazeSolver(int scene_number);
  PIDMazeSolver(int scene_number,
                const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  struct PoseOrient { // ❗ Renamed from `Waypoint`
    float x{0.0f};
    float y{0.0f};
    float yaw{0.0f};

    PoseOrient() = default;
    PoseOrient(float x_, float y_, float yaw_) : x(x_), y(y_), yaw(yaw_) {}
  };

  enum class State {
    INITIAL, // Home or Starting position
    TURN,    // Turn Towards next waypoint
    MOVE,    // Head Towards next waypoint
    FINAL    // Final position
  };

  struct Opening {
    double center_angle_deg; // robot-frame degrees from front (0=forward)
    double width_deg;        // angular width of the gap
    double avg_range;        // average range inside the gap (m)
  };

private:
  struct LaserDirections {
    int front_index;
    int back_index;
    int left_index;
    int right_index;
  };

  // === Member variables ===
  LaserDirections laser_dirs_;
  Laser::SharedPtr last_scan_;

  float TkP_;
  float TkI_;
  float TkD_;
  float DkP_;
  float DkI_;
  float DkD_;
  float max_ang_vel_;
  float odom_drift_threshold_;
  float yaw_tolerance_;

  bool initial_odom_received_ = false;
  bool relative_mode_ = false;
  PoseOrient current_pose_;
  std::vector<PoseOrient> all_waypoints_;
  std::vector<size_t> fwd_waypoint_indices_seq_;
  std::vector<size_t> rev_waypoint_indices_seq_;
  std::vector<size_t> combined_waypoint_indices_seq_;
  std::vector<float> execution_yaws_;
  std::vector<int> waypoint_sequence_; // indices to visit

  int num_waypoints_;
  int scene_number_;
  std::string odom_topic_;
  std::string laser_topic_;

  rclcpp::Publisher<Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  rclcpp::Subscription<Laser>::SharedPtr laser_sub_;

  rclcpp::SubscriptionOptions odom_sub_options_;
  rclcpp::SubscriptionOptions laser_sub_options_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Laser config (computed once in initLaser)
  int front_idx_;
  int north_idx_, south_idx_, east_idx_, west_idx_;
  int ne_idx_, nw_idx_, se_idx_, sw_idx_;
  double dN, dS, dE, dW, dNE, dNW, dSE, dSW;
  int band_half_; // ± ray count around each direction
  double angle_increment_;
  double angle_min_;
  int num_rays_;
  bool laser_initialized_{false};
  double laser_offset_deg_;
  bool laser_flip_{false}; // set true in sim if N↔S and E↔W are swapped

  // Tunable — set via declare_parameter or constructor
  double WALL_CLOSE_THRESH = 0.25; // (m) warn if wall closer than this
  double BAND_DEGREES = 10.0;      // ± degrees for averaging band
  double OPENING_MIN_RANGE = 0.50; // (m) ray longer than this → open
  double OPENING_MIN_DEG = 20.0;   // minimum angular width to count as opening
  double MAX_RANGE_CLIP = 3.50;    // clip inf readings to this

  std::vector<Opening> openings_; // populated each callback
  void initLaser(const Laser::SharedPtr msg);
  static double bandAvg(const std::vector<float> &ranges, int center, int half,
                        double clip, float range_max);

  State state_{State::INITIAL};
  void get_homepostion();
  void face_next_wp();
  void head_to_next_wp();

  void LoadParameters();
  void LoadWaypointsYaml();
  void BuildExecutionYaws();
  LaserDirections compute_laser_indices(const Laser &scan);
  void laserCallback_old(const Laser::SharedPtr msg);
  void laserCallback(const Laser::SharedPtr msg);
  void timer_callback();
  void StopRobot();
  void waypoint_selection();
  void printWaypointsSequence(const std::vector<size_t> &seq,
                              const std::string &name) const;
  // std::string prnt_waypoints() const;
  std::string prnt_waypoints(const std::vector<PoseOrient> &vec) const;
  static float NormalizeAngle(float angle);
};

inline std::ostream &operator<<(std::ostream &os,
                                const PIDMazeSolver::PoseOrient &p) {
  os << "(x=" << p.x << ", y=" << p.y << ", yaw=" << p.yaw << " rad)";
  return os;
}

inline std::ostream &
operator<<(std::ostream &os,
           const std::vector<PIDMazeSolver::PoseOrient> &vec) {

  os << "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    os << vec[i];
    if (i != vec.size() - 1)
      os << ", ";
  }
  os << "]";
  return os;
}