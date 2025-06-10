#pragma once

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "arm_controller/KinovaArmController.h"

class TrajectoryControllerNode : public rclcpp::Node
{
public:
    TrajectoryControllerNode();

private:
    void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

    KinovaArmController arm_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
};
