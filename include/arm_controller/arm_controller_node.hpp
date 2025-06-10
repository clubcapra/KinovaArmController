#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "arm_controller/KinovaArmController.h"

class ArmControllerNode : public rclcpp::Node
{
public:
    ArmControllerNode();

private:
    // Periodic timer callback
    void timer_callback();

    // Helper publishers
    void publish_float64_multi_array(
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub,
        const std::vector<double>& data);

    void publish_float64(
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub,
        double value);

    // ---- Members ----
    rclcpp::TimerBase::SharedPtr timer_;
    KinovaArmController arm_;

    // Array publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr positions_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocities_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torques_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr currents_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr temperatures_pub_;

    // Per-joint publishers (position, velocity, torque, current, temperature)
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> per_joint_position_pubs_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> per_joint_velocity_pubs_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> per_joint_torque_pubs_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> per_joint_current_pubs_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> per_joint_temperature_pubs_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr set_positions_sub_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> set_position_subs_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr set_velocities_sub_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> set_velocity_subs_;

    // Callback handlers
    void set_positions_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void set_position_callback(const std_msgs::msg::Float64::SharedPtr msg, int idx);

    void set_velocities_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void set_velocity_callback(const std_msgs::msg::Float64::SharedPtr msg, int idx);

    // Names for standard joint_state message
    const std::vector<std::string> joint_names_ = {
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    };
};
