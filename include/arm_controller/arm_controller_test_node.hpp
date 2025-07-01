#ifndef ARM_CONTROLLER_TEST_NODE_HPP
#define ARM_CONTROLLER_TEST_NODE_HPP

#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "arm_controller/KinovaArmControllerTest.h"

class ArmControllerTestNode : public rclcpp::Node
{
public:
    ArmControllerTestNode();

private:
    // Callbacks
    void set_positions_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void set_velocities_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void timer_callback();

    // Publishing helpers
    void publish_multi(
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub,
        const std::vector<double>& data);
    void publish_single(
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub,
        double value);

    // Test arm controller
    KinovaArmControllerTest arm_;

    // Publishers for all-joint arrays
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr positions_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocities_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torques_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr currents_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr temperatures_pub_;

    // Per-joint publishers
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> per_joint_position_pubs_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> per_joint_velocity_pubs_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> per_joint_torque_pubs_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> per_joint_current_pubs_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> per_joint_temperature_pubs_;

    // Subscribers for set commands
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr set_positions_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr set_velocities_sub_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> set_position_subs_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> set_velocity_subs_;

    // Timer for periodic state publishing
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // ARM_CONTROLLER_TEST_NODE_HPP
