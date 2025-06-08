#pragma once

#include "KinovaArmController.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mutex>
#include <vector>
#include <string>

class ArmControllerNode : public rclcpp::Node {
public:
    ArmControllerNode();

private:
    // ---- Command interfaces
    void jointStateCmdCb(const sensor_msgs::msg::JointState::SharedPtr msg);

    // ---- Action: Trajectory execution
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr traj_action_server_;
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid,std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle);
    void handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle);
    void executeTrajectory(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle);

    // ---- Service: Home
    void sendHomeService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    // ---- Feedback Publishing (Timer)
    void publishFeedback();

    // ---- Feedback topics (all joints)
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_currents_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_temps_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_torques_pub_;
    rclcpp::TimerBase::SharedPtr feedback_timer_;

    // ---- Feedback topics (per joint)
    std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> temp_pubs_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> current_pubs_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> torque_pubs_;

    // ---- Command subscription
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_cmd_sub_;

    // ---- Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_srv_;

    // ---- Low-level driver
    KinovaArmController arm_;
    std::mutex arm_mutex_;
};
