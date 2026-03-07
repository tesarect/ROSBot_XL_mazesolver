#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <cmath>
#include <string>

class LaserDiagnostic : public rclcpp::Node {
public:
  LaserDiagnostic() : Node("laser_diagnostic") {

    // TF listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Laser scan subscriber
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          scan_callback(msg);
        });

    // Timer: try to read TF every 1s until we get it
    tf_timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                        [this]() { try_lookup_tf(); });

    RCLCPP_INFO(this->get_logger(),
                "LaserDiagnostic started. Waiting for TF...");
  }

private:
  // TF Lookup
  void try_lookup_tf() {
    if (tf_ready_)
      return; // already have it

    try {
      // Get transform from base_link to laser link, &
      // Get the laser orientation wrt base_link
      geometry_msgs::msg::TransformStamped tf =
          tf_buffer_->lookupTransform("base_link", "laser", tf2::TimePointZero);

      // Extract yaw from the rotation quaternion
      tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                        tf.transform.rotation.z, tf.transform.rotation.w);

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      laser_mount_yaw_ = static_cast<float>(yaw);
      tf_ready_ = true;
      tf_timer_->cancel(); // no need to keep polling

      RCLCPP_INFO(this->get_logger(), "\nTF base_link → laser found:");
      RCLCPP_INFO(this->get_logger(), "  translation : x=%.4f  y=%.4f  z=%.4f",
                  tf.transform.translation.x, tf.transform.translation.y,
                  tf.transform.translation.z);
      RCLCPP_INFO(this->get_logger(),
                  "  rotation    : roll=%.4f  pitch=%.4f  yaw=%.4f rad", roll,
                  pitch, yaw);
      RCLCPP_INFO(this->get_logger(), "  yaw (deg)   : %.2f°",
                  yaw * 180.0 / M_PI);
      RCLCPP_INFO(this->get_logger(), "  → laser_mount_yaw = %.4f rad (%.2f°)",
                  laser_mount_yaw_, laser_mount_yaw_ * 180.0f / M_PI);

    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "TF not yet available: %s  (retrying...)", e.what());
    }
  }

  // read ranges for 8 directions
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!tf_ready_)
      return;

    struct Direction {
      std::string name;
      float robot_angle_rad;
    };

    const std::vector<Direction> directions = {
        {"N   ", 0.0f},
        {"NE  ", -M_PI / 4.0f},
        {"E   ", -M_PI / 2.0f},
        {"SE  ", -3.0f * M_PI / 4.0f},
        {"S   ", static_cast<float>(M_PI)},
        {"SW  ", 3.0f * M_PI / 4.0f},
        {"W   ", M_PI / 2.0f},
        {"NW  ", M_PI / 4.0f},
    };

    // ±10 degrees window around each direction
    constexpr float WINDOW_DEG = 10.0f;
    constexpr float WINDOW_RAD = WINDOW_DEG * M_PI / 180.0f;

    for (const auto &dir : directions) {
      float laser_angle =
          normalize_angle(dir.robot_angle_rad - laser_mount_yaw_);
      float range = get_avg_range(msg, laser_angle, WINDOW_RAD);

      if (std::isinf(range)) {
        RCLCPP_INFO(this->get_logger(), "  %s | center=%6.2f° | range=  inf",
                    dir.name.c_str(), laser_angle * 180.0f / M_PI);
      } else {
        RCLCPP_INFO(this->get_logger(), "  %s | center=%6.2f° | range=%6.3f m",
                    dir.name.c_str(), laser_angle * 180.0f / M_PI, range);
      }
    }
  }

  // average valid ranges within +/-half_window of laser_angle
  // Scans all indices whose angle falls within [laser_angle - half_window,
  // laser_angle + half_window], collects valid readings, returns their mean.
  // Returns inf if no valid readings found in the window.
  float get_avg_range(const sensor_msgs::msg::LaserScan::SharedPtr &msg,
                      float laser_angle, float half_window) {
    float angle_min = msg->angle_min;
    float angle_inc = msg->angle_increment;
    int n = static_cast<int>(msg->ranges.size());

    if (angle_inc == 0.0f || n == 0)
      return std::numeric_limits<float>::infinity();

    // Find index range covering [laser_angle - half_window, laser_angle +
    // half_window]
    float low = normalize_angle(laser_angle - half_window);
    float high = normalize_angle(laser_angle + half_window);

    float sum = 0.0f;
    int count = 0;

    for (int i = 0; i < n; i++) {
      float angle = normalize_angle(angle_min + i * angle_inc);

      // Check if this angle falls within the window
      // Need to handle wrap-around at ±π
      bool in_window = false;
      if (low <= high) {
        in_window = (angle >= low && angle <= high);
      } else {
        // window wraps around ±π boundary (e.g. S direction at ±180°)
        in_window = (angle >= low || angle <= high);
      }

      if (!in_window)
        continue;

      float r = msg->ranges[i];
      if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
        sum += r;
        count++;
      }
    }

    if (count == 0)
      return std::numeric_limits<float>::infinity();

    return sum / static_cast<float>(count);
  }

  float normalize_angle(float angle) {
    while (angle > M_PI)
      angle -= 2.0f * M_PI;
    while (angle < -M_PI)
      angle += 2.0f * M_PI;
    return angle;
  }

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  float laser_mount_yaw_ = 0.0f;
  bool tf_ready_ = false;

  // Scan
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserDiagnostic>());
  rclcpp::shutdown();
  return 0;
}