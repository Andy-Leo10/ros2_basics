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
    posX = data_odom_->pose.pose.position.x;
    posY = data_odom_->pose.pose.position.y;
    // RCLCPP_INFO(this->get_logger(), "posX: '%f'",posX);
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    data_laser_ = msg;
    range0 = data_laser_->ranges[0];
    range180 = data_laser_->ranges[180];
    range360 = data_laser_->ranges[360];
    range540 = data_laser_->ranges[540];
    range719 = data_laser_->ranges[719];
    RCLCPP_INFO(this->get_logger(), "range360: '%f'", range360);
  }

  void timer_callback()
  {
    // If the laser reading in front of the robot is higher than one meter the robot will move forward, else it will go to the left
    if (range360 > 1.0)
    {
      move.linear.x = 0.5;
      move.angular.z = 0.0;
    }
    else
    {
      move.angular.z = 0.5;
    }
    // If the laser reading on the right side of the robot is lower than one meter the robot will turn left
    if (range180 < 1.0)
    {
      move.angular.z = 0.5;
    }
    // If the laser reading on the left side of the robot is lower than one meter the robot will turn right
    if (range540 < 1.0)
    {
      move.angular.z = -0.5;
    }
    publisher_->publish(move);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  nav_msgs::msg::Odometry::SharedPtr data_odom_;
  float posX;
  float posY;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
  sensor_msgs::msg::LaserScan::SharedPtr data_laser_;
  float range0;
  float range180;
  float range360;
  float range540;
  float range719;
  rclcpp::TimerBase::SharedPtr timer_;
  // create a message to publish
  geometry_msgs::msg::Twist move;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotNeo>());
  rclcpp::shutdown();
  return 0;
}
