#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

class RobotNeo : public rclcpp::Node
{
public:
  RobotNeo() : Node("robot_neo_node")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&RobotNeo::odom_callback, this, _1));
    laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&RobotNeo::laser_callback, this, _1));
    timer_ = this->create_wall_timer(500ms, std::bind(&RobotNeo::timer_callback, this));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    data_odom_ = msg;
    RCLCPP_INFO(this->get_logger(), "posX: '%f'",
                data_odom_->pose.pose.position.x);
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    data_laser_ = msg;
    RCLCPP_INFO(this->get_logger(), "range0: '%f'",
                data_laser_->ranges[0]);
  }

  void timer_callback()
  {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.2;
    message.angular.z = 0.2;
    publisher_->publish(message);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  nav_msgs::msg::Odometry::SharedPtr data_odom_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
  sensor_msgs::msg::LaserScan::SharedPtr data_laser_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotNeo>());
  rclcpp::shutdown();
  return 0;
}
