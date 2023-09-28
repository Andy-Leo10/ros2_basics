#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;

class OdomSubscriber : public rclcpp::Node {
public:
  OdomSubscriber() : Node("simple_subscriber") {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&OdomSubscriber::odom_callback, this, _1));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    data_odom_ = msg;
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'",
                data_odom_->pose.pose.position.x);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  nav_msgs::msg::Odometry::SharedPtr data_odom_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomSubscriber>());
  rclcpp::shutdown();
  return 0;
}



#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

using namespace std::chrono_literals;

class MoveRobot : public rclcpp::Node {
public:
  MoveRobot() : Node("move_robot") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MoveRobot::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.2;
    message.angular.z = 0.2;
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveRobot>());
  rclcpp::shutdown();
  return 0;
}
