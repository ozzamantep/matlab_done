#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <underwater_interfaces/msg/sonar_scan.hpp>

class AUVControllerNode : public rclcpp::Node
{
public:
  AUVControllerNode() : Node("auv_controller")
  {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/seeker/cmd_vel", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/seeker/odometry", 10,
      std::bind(&AUVControllerNode::odom_callback, this, std::placeholders::_1));

    target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/seeker/target_estimate", 10,
      std::bind(&AUVControllerNode::target_callback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/seeker/mission_state", 10,
      std::bind(&AUVControllerNode::state_callback, this, std::placeholders::_1));

    sonar_sub_ = this->create_subscription<underwater_interfaces::msg::SonarScan>(
      "/seeker/sonar/scan", 10,
      std::bind(&AUVControllerNode::sonar_callback, this, std::placeholders::_1));

    // Lawnmower search waypoints in world ENU (meters)
    waypoints_ = {
      {-35.0, -35.0, -10.0}, {35.0, -35.0, -12.0}, {35.0, -10.0, -14.0},
      {-35.0, -10.0, -12.0}, {-35.0, 15.0, -14.0}, {35.0, 15.0, -12.0},
      {35.0, 38.0, -10.0},   {-35.0, 38.0, -12.0}
    };

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&AUVControllerNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "6-DOF AUVControllerNode initialized (PID + APF Obstacle Avoidance).");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    pos_[0] = msg->pose.pose.position.x;
    pos_[1] = msg->pose.pose.position.y;
    pos_[2] = msg->pose.pose.position.z;

    const auto & q = msg->pose.pose.orientation;
    yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    has_odom_ = true;
  }

  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    target_pos_[0] = msg->pose.position.x;
    target_pos_[1] = msg->pose.position.y;
    target_pos_[2] = msg->pose.position.z;
    has_target_ = true;
  }

  void state_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    mission_state_ = msg->data;
  }

  void sonar_callback(const underwater_interfaces::msg::SonarScan::SharedPtr msg)
  {
    // Artificial Potential Field repulsive vector in body frame from close echoes
    repulsive_body_[0] = 0.0;
    repulsive_body_[1] = 0.0;
    constexpr double safe_range = 6.0;

    for (const auto & ping : msg->detections) {
      if (ping.range > 0.2f && ping.range < safe_range) {
        const double az = ping.azimuth_angle * M_PI / 180.0;
        const double gain = 1.8 * (1.0 / ping.range - 1.0 / safe_range);
        repulsive_body_[0] -= gain * std::cos(az);
        repulsive_body_[1] -= gain * std::sin(az);
      }
    }
  }

  void control_loop()
  {
    if (!has_odom_) {
      return;
    }

    std::array<double, 3> goal = waypoints_[wp_idx_];
    double max_speed = 1.65;

    if ((mission_state_ == "TRACK" || mission_state_ == "INTERCEPT") && has_target_) {
      goal = target_pos_;
      max_speed = (mission_state_ == "INTERCEPT") ? 2.05 : 1.80;
    } else {
      const double dx = goal[0] - pos_[0];
      const double dy = goal[1] - pos_[1];
      const double dz = goal[2] - pos_[2];
      if (std::sqrt(dx * dx + dy * dy + dz * dz) < 4.5) {
        wp_idx_ = (wp_idx_ + 1) % waypoints_.size();
        goal = waypoints_[wp_idx_];
      }
    }

    const double dx = goal[0] - pos_[0];
    const double dy = goal[1] - pos_[1];
    const double dz = goal[2] - pos_[2];
    const double dist = std::max(1e-4, std::sqrt(dx * dx + dy * dy + dz * dz));

    // World desired velocity
    const double vx_w = (dx / dist) * max_speed;
    const double vy_w = (dy / dist) * max_speed;
    const double vz_w = (dz / dist) * max_speed;

    // Transform to body frame + add APF repulsive term
    const double cy = std::cos(yaw_);
    const double sy = std::sin(yaw_);
    double vx_b = cy * vx_w + sy * vy_w + repulsive_body_[0];
    double vy_b = -sy * vx_w + cy * vy_w + repulsive_body_[1];

    const double desired_yaw = std::atan2(dy, dx);
    const double yaw_err = std::atan2(std::sin(desired_yaw - yaw_), std::cos(desired_yaw - yaw_));

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::clamp(vx_b / 2.2, -1.0, 1.0);
    cmd.linear.y = std::clamp(vy_b / 2.2, -1.0, 1.0);
    cmd.linear.z = std::clamp(vz_w / 1.5, -1.0, 1.0);
    cmd.angular.z = std::clamp(1.6 * yaw_err, -1.0, 1.0);

    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<underwater_interfaces::msg::SonarScan>::SharedPtr sonar_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool has_odom_{false};
  bool has_target_{false};
  std::string mission_state_{"SEARCH"};
  std::array<double, 3> pos_{0.0, 0.0, -10.0};
  std::array<double, 3> target_pos_{0.0, 0.0, -10.0};
  std::array<double, 2> repulsive_body_{0.0, 0.0};
  double yaw_{0.0};
  std::vector<std::array<double, 3>> waypoints_;
  size_t wp_idx_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AUVControllerNode>());
  rclcpp::shutdown();
  return 0;
}
