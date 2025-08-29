#include "magni_webots/MagniDriver.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>


#define HALF_DISTANCE_BETWEEN_WHEELS 0.163
#define WHEEL_RADIUS 0.1

namespace magni_webots {
  void MagniDriver::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
  {
    //get reference from webots
    right_motor = wb_robot_get_device("right_wheel_joint");
    left_motor = wb_robot_get_device("left_wheel_joint");

    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(right_motor, 0);

    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_velocity(left_motor, INFINITY);

    cmd_vel_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SensorDataQoS().reliable(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg){
        this->cmd_vel_msg.linear = msg->linear;
        this->cmd_vel_msg.angular = msg->angular;
      }
    );
  }

  void MagniDriver::step()
  {
    auto forward_speed = cmd_vel_msg.linear.x;
    auto angular_speed = cmd_vel_msg.angular.z;

    auto command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS)/ WHEEL_RADIUS;
    auto command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS)/ WHEEL_RADIUS;

    wb_motor_set_velocity(left_motor, command_motor_left);
    wb_motor_set_velocity(right_motor, command_motor_right);
  }
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(magni_webots::MagniDriver, webots_ros2_driver::PluginInterface)
