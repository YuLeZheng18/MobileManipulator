#include <array>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace {
constexpr double kWheelRadius = 0.0635;
constexpr double kHalfLength = 0.20;
constexpr double kHalfWidth = 0.20;
}

class OmniKinematicsNode : public rclcpp::Node {
public:
  OmniKinematicsNode() : Node("omni_kinematics") {
    using std::placeholders::_1;

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&OmniKinematicsNode::cmdCallback, this, _1));

    front_left_wheel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/front_left_wheel_controller/commands", 10);
    front_right_wheel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/front_right_wheel_controller/commands", 10);
    rear_left_wheel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/rear_left_wheel_controller/commands", 10);
    rear_right_wheel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/rear_right_wheel_controller/commands", 10);
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    const double l_plus_w = kHalfLength + kHalfWidth;

    const double front_left = (msg->linear.x - msg->linear.y - l_plus_w * msg->angular.z) / kWheelRadius;
    const double front_right = (msg->linear.x + msg->linear.y + l_plus_w * msg->angular.z) / kWheelRadius;
    const double rear_left = (msg->linear.x + msg->linear.y - l_plus_w * msg->angular.z) / kWheelRadius;
    const double rear_right = (msg->linear.x - msg->linear.y + l_plus_w * msg->angular.z) / kWheelRadius;

    publishCommand(front_left_wheel_pub_, front_left);
    publishCommand(front_right_wheel_pub_, front_right);
    publishCommand(rear_left_wheel_pub_, rear_left);
    publishCommand(rear_right_wheel_pub_, rear_right);
  }

  void publishCommand(const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr & publisher,
                      double value) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.push_back(value);
    publisher->publish(msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr front_left_wheel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr front_right_wheel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rear_left_wheel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rear_right_wheel_pub_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OmniKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
