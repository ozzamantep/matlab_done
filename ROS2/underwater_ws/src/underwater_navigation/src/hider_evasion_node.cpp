#include <algorithm>
#include <array>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

class HiderEvasionNode : public rclcpp::Node
{
public:
  HiderEvasionNode() : Node("hider_evasion_node")
  {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/hider/cmd_vel", 10);

    hider_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/hider/odometry", 10,
      std::bind(&HiderEvasionNode::hider_odom_cb, this, std::placeholders::_1));

    seeker_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/seeker/odometry", 10,
      std::bind(&HiderEvasionNode::seeker_odom_cb, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&HiderEvasionNode::evasion_loop, this));

    RCLCPP_INFO(this->get_logger(), "HiderEvasionNode initialized.");
  }

private:
  void hider_odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    hider_pos_ = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
    const auto & q = msg->pose.pose.orientation;
    hider_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    has_hider_ = true;
  }

  void seeker_odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    seeker_pos_ = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
    has_seeker_ = true;
  }

  void evasion_loop()
  {
    if (!has_hider_ || !has_seeker_) {
      return;
    }

    const double dx = hider_pos_[0] - seeker_pos_[0];
    const double dy = hider_pos_[1] - seeker_pos_[1];
    const double dist = std::max(1e-4, std::hypot(dx, dy));

    const double t = this->now().seconds();
    double vx_w = (dx / dist) * 1.45 + 0.5 * (-dy / dist) * std::sin(0.35 * t);
    double vy_w = (dy / dist) * 1.45 + 0.5 * (dx / dist) * std::sin(0.35 * t);

    // Arena boundary repulsion (+/- 50m)
    if (hider_pos_[0] < -50.0) vx_w = std::abs(vx_w);
    if (hider_pos_[0] >  50.0) vx_w = -std::abs(vx_w);
    if (hider_pos_[1] < -50.0) vy_w = std::abs(vy_w);
    if (hider_pos_[1] >  50.0) vy_w = -std::abs(vy_w);

    const double cy = std::cos(hider_yaw_);
    const double sy = std::sin(hider_yaw_);
    const double vx_b = cy * vx_w + sy * vy_w;
    const double vy_b = -sy * vx_w + cy * vy_w;

    const double des_yaw = std::atan2(vy_w, vx_w);
    const double yaw_err = std::atan2(std::sin(des_yaw - hider_yaw_), std::cos(des_yaw - hider_yaw_));

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::clamp(vx_b / 2.0, -1.0, 1.0);
    cmd.linear.y = std::clamp(vy_b / 2.0, -1.0, 1.0);
    cmd.linear.z = std::clamp((-14.0 - hider_pos_[2]) * 0.25, -0.5, 0.5);
    cmd.angular.z = std::clamp(1.4 * yaw_err, -1.0, 1.0);

    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr hider_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr seeker_odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool has_hider_{false};
  bool has_seeker_{false};
  std::array<double, 3> hider_pos_{-12.0, -10.0, -14.0};
  std::array<double, 3> seeker_pos_{30.0, 30.0, -10.0};
  double hider_yaw_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HiderEvasionNode>());
  rclcpp::shutdown();
  return 0;
}
