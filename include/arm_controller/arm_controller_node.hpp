#ifndef ARM_CONTROLLER_NODE_HPP
#define ARM_CONTROLLER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "KinovaArmController.h"
#include <std_msgs/msg/float64_multi_array.hpp>

class ArmControllerNode : public rclcpp::Node
{
public:
  ArmControllerNode();

private:
  KinovaArmController arm_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;
};

#endif
