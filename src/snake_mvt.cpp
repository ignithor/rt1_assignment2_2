#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class SnakeMvtNode : public rclcpp::Node
{
public:
  SnakeMvtNode() : Node("snake_mvt_node")
  {
    // Publisher to publish Twist messages to /cmd_vel topic
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // Subscription to /odom topic to get the position of the robot
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&SnakeMvtNode::odom_callback, this, std::placeholders::_1));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto twist = geometry_msgs::msg::Twist();
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    // If the robot is out of the map, stop the robot 
    // else move the robot depending on its position
    if (y <1.0 || y>9.0)
    {
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
    }
    else if (x > 8.5)
    {
      twist.linear.x = 1.0;
      twist.angular.z = 2.0;
    }
    else if (x < 2.0)
    {
      twist.linear.x = 1.0;
      twist.angular.z = -2.0;
    }
    else
    {
      twist.linear.x = 1.0;
      twist.angular.z = 0.0;
    }

    publisher_->publish(twist);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SnakeMvtNode>());
  rclcpp::shutdown();
  return 0;
}
