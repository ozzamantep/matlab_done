#include <cmath>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <underwater_interfaces/msg/sonar_scan.hpp>

class SonarTargetTrackerNode : public rclcpp::Node
{
public:
  SonarTargetTrackerNode() : Node("sonar_target_tracker")
  {
    sonar_sub_ = this->create_subscription<underwater_interfaces::msg::SonarScan>(
      "/seeker/sonar/scan", 10,
      std::bind(&SonarTargetTrackerNode::sonar_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/seeker/odometry", 10,
      std::bind(&SonarTargetTrackerNode::odom_callback, this, std::placeholders::_1));

    target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/seeker/target_estimate", 10);
    state_pub_ = this->create_publisher<std_msgs::msg::String>("/seeker/mission_state", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&SonarTargetTrackerNode::timer_callback, this));

    last_meas_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "SonarTargetTrackerNode initialized (EKF + Search/Track/Intercept).");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    seeker_pos_[0] = msg->pose.pose.position.x;
    seeker_pos_[1] = msg->pose.pose.position.y;
    seeker_pos_[2] = msg->pose.pose.position.z;

    const auto & q = msg->pose.pose.orientation;
    seeker_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    has_odom_ = true;
  }

  void sonar_callback(const underwater_interfaces::msg::SonarScan::SharedPtr msg)
  {
    if (!has_odom_ || msg->detections.empty()) {
      return;
    }

    // Select strongest echo return (filtered target ping)
    const auto * best_ping = &msg->detections[0];
    for (const auto & ping : msg->detections) {
      if (ping.intensity > best_ping->intensity) {
        best_ping = &ping;
      }
    }

    if (best_ping->intensity < 0.2f || best_ping->range > msg->max_range) {
      return;
    }

    const double az = best_ping->azimuth_angle * M_PI / 180.0;
    const double el = best_ping->elevation_angle * M_PI / 180.0;
    const double r = best_ping->range;

    // Body-frame spherical to world ENU
    const double bx = r * std::cos(el) * std::cos(az);
    const double by = r * std::cos(el) * std::sin(az);
    const double bz = r * std::sin(el);

    const double cy = std::cos(seeker_yaw_);
    const double sy = std::sin(seeker_yaw_);
    const double wx = seeker_pos_[0] + cy * bx - sy * by;
    const double wy = seeker_pos_[1] + sy * bx + cy * by;
    const double wz = seeker_pos_[2] + bz;

    // Alpha-Beta / EKF measurement update
    if (!ekf_initialized_) {
      est_pos_[0] = wx;
      est_pos_[1] = wy;
      est_pos_[2] = wz;
      est_vel_[0] = est_vel_[1] = est_vel_[2] = 0.0;
      ekf_initialized_ = true;
    } else {
      constexpr double alpha = 0.45;
      constexpr double beta = 0.15;
      const double dt = std::max(0.02, (this->now() - last_meas_time_).seconds());
      for (int i = 0; i < 3; ++i) {
        const double meas = (i == 0) ? wx : ((i == 1) ? wy : wz);
        const double innov = meas - est_pos_[i];
        est_pos_[i] += alpha * innov;
        est_vel_[i] = 0.9 * est_vel_[i] + (beta / dt) * innov;
      }
    }
    last_meas_time_ = this->now();
  }

  void timer_callback()
  {
    constexpr double dt = 0.05;
    std_msgs::msg::String state_msg;

    if (!ekf_initialized_) {
      state_msg.data = "SEARCH";
      state_pub_->publish(state_msg);
      return;
    }

    // Constant-velocity EKF prediction
    for (int i = 0; i < 3; ++i) {
      est_pos_[i] += est_vel_[i] * dt;
      est_vel_[i] *= 0.985;
    }

    const double age = (this->now() - last_meas_time_).seconds();
    const double dx = est_pos_[0] - seeker_pos_[0];
    const double dy = est_pos_[1] - seeker_pos_[1];
    const double dz = est_pos_[2] - seeker_pos_[2];
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (age > 6.0) {
      state_msg.data = "SEARCH";
    } else if (dist < 16.0) {
      state_msg.data = "INTERCEPT";
    } else {
      state_msg.data = "TRACK";
    }
    state_pub_->publish(state_msg);

    geometry_msgs::msg::PoseStamped target_msg;
    target_msg.header.stamp = this->now();
    target_msg.header.frame_id = "world";
    target_msg.pose.position.x = est_pos_[0];
    target_msg.pose.position.y = est_pos_[1];
    target_msg.pose.position.z = est_pos_[2];
    target_msg.pose.orientation.w = 1.0;
    target_pub_->publish(target_msg);
  }

  rclcpp::Subscription<underwater_interfaces::msg::SonarScan>::SharedPtr sonar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool has_odom_{false};
  bool ekf_initialized_{false};
  double seeker_pos_[3]{0.0, 0.0, -10.0};
  double seeker_yaw_{0.0};
  double est_pos_[3]{0.0, 0.0, -10.0};
  double est_vel_[3]{0.0, 0.0, 0.0};
  rclcpp::Time last_meas_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SonarTargetTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
