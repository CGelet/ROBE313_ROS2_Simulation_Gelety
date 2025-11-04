#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "custom_interfaces/srv/reset_position.hpp"

using namespace std::chrono_literals;

class OdometryNode : public rclcpp::Node
{
public:
  OdometryNode()
  : Node("odometry_node"),
    x_(0.0), y_(0.0), theta_(0.0),
    v_(0.0), omega_(0.0)
  {
    // Subscribe to /cmd_vel
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&OdometryNode::cmdVelCallback, this, std::placeholders::_1));

    // ResetPosition service
    reset_srv_ = this->create_service<custom_interfaces::srv::ResetPosition>(
      "/ResetPosition",
      std::bind(&OdometryNode::resetCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    last_time_ = this->now();
    timer_ = this->create_wall_timer(
      20ms, std::bind(&OdometryNode::update, this));  // 50 Hz

    RCLCPP_INFO(this->get_logger(), "Odometry node started");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    v_     = msg->linear.x;
    omega_ = msg->angular.z;
  }

  void resetCallback(
    const custom_interfaces::srv::ResetPosition::Request::SharedPtr req,
    const custom_interfaces::srv::ResetPosition::Response::SharedPtr res)
  {
    // Extract x,y from Pose
    x_ = req->pose.position.x;
    y_ = req->pose.position.y;

    // Extract yaw from quaternion (simplified; assumes flat robot)
    const auto & q = req->pose.orientation;
    // yaw from quaternion (standard formula)
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    theta_ = std::atan2(siny_cosp, cosy_cosp);

    RCLCPP_INFO(this->get_logger(),
      "Resetting pose to x=%.2f, y=%.2f, theta=%.2f rad", x_, y_, theta_);

    res->success = true;
  }

  void update()
  {
    auto now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    // Integrate pose
    x_     += v_ * dt * std::cos(theta_);
    y_     += v_ * dt * std::sin(theta_);
    theta_ += omega_ * dt;

    // Normalize theta_ a bit (optional)
    if (theta_ > M_PI)  theta_ -= 2.0 * M_PI;
    if (theta_ < -M_PI) theta_ += 2.0 * M_PI;

    // Broadcast odom -> base_link
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id  = "base_link";

    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
  }

  // Members
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Service<custom_interfaces::srv::ResetPosition>::SharedPtr reset_srv_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;

  double x_, y_, theta_;
  double v_, omega_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryNode>());
  rclcpp::shutdown();
  return 0;
}
