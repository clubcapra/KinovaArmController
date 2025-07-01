#include "arm_controller/arm_controller_test_node.hpp"
#include "arm_controller/KinovaArmControllerTest.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

ArmControllerTestNode::ArmControllerTestNode()
: Node("arm_controller_test_node"), arm_()
{
    // All joints array publishers
    positions_pub_    = create_publisher<std_msgs::msg::Float64MultiArray>("kinova_arm/joints/positions", 10);
    velocities_pub_   = create_publisher<std_msgs::msg::Float64MultiArray>("kinova_arm/joints/velocities", 10);
    torques_pub_      = create_publisher<std_msgs::msg::Float64MultiArray>("kinova_arm/joints/torques", 10);
    currents_pub_     = create_publisher<std_msgs::msg::Float64MultiArray>("kinova_arm/joints/currents", 10);
    temperatures_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("kinova_arm/joints/temperatures", 10);

    // Per-joint publishers
    for (size_t i = 0; i < 6; ++i)
    {
        const auto num = std::to_string(i+1);
        per_joint_position_pubs_.push_back(
            create_publisher<std_msgs::msg::Float64>("kinova_arm/joint" + num + "/position", 10));
        per_joint_velocity_pubs_.push_back(
            create_publisher<std_msgs::msg::Float64>("kinova_arm/joint" + num + "/velocity", 10));
        per_joint_torque_pubs_.push_back(
            create_publisher<std_msgs::msg::Float64>("kinova_arm/joint" + num + "/torque", 10));
        per_joint_current_pubs_.push_back(
            create_publisher<std_msgs::msg::Float64>("kinova_arm/joint" + num + "/current", 10));
        per_joint_temperature_pubs_.push_back(
            create_publisher<std_msgs::msg::Float64>("kinova_arm/joint" + num + "/temperature", 10));
    }

    timer_ = create_wall_timer(25ms, std::bind(&ArmControllerTestNode::timer_callback, this));

    // Set positions (all)
    set_positions_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "kinova_arm/joints/setPositions", 10,
        std::bind(&ArmControllerTestNode::set_positions_callback, this, std::placeholders::_1)
    );
    // Set velocities (all)
    set_velocities_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "kinova_arm/joints/setVelocities", 10,
        std::bind(&ArmControllerTestNode::set_velocities_callback, this, std::placeholders::_1)
    );

    // Per-joint set subs
    for (size_t i = 0; i < 6; ++i) {
        auto pos_sub = create_subscription<std_msgs::msg::Float64>(
            "kinova_arm/joint" + std::to_string(i+1) + "/setPosition", 10,
            [this, i](std_msgs::msg::Float64::SharedPtr msg) {
                arm_.sendJointPosition(i, msg->data);
            }
        );
        set_position_subs_.push_back(pos_sub);

        auto vel_sub = create_subscription<std_msgs::msg::Float64>(
            "kinova_arm/joint" + std::to_string(i+1) + "/setVelocity", 10,
            [this, i](std_msgs::msg::Float64::SharedPtr msg) {
                arm_.sendJointVelocity(i, msg->data);
            }
        );
        set_velocity_subs_.push_back(vel_sub);
    }
}

void ArmControllerTestNode::set_positions_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    arm_.sendJointsPosition(msg->data);
}

void ArmControllerTestNode::set_velocities_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    std::vector<float> vels(msg->data.begin(), msg->data.end());
    arm_.sendJointsVelocity(vels);
}

void ArmControllerTestNode::timer_callback() {
    auto positions    = arm_.getJointsPositions();
    auto velocities   = arm_.getJointsVelocities();
    auto torques      = arm_.getJointsTorques();
    auto currents     = arm_.getJointsCurrents();
    auto temperatures = arm_.getJointsTemperatures();

    publish_multi(positions_pub_,    positions);
    publish_multi(velocities_pub_,   std::vector<double>(velocities.begin(), velocities.end()));
    publish_multi(torques_pub_,      torques);
    publish_multi(currents_pub_,     currents);
    publish_multi(temperatures_pub_, std::vector<double>(temperatures.begin(), temperatures.end()));

    for (size_t i = 0; i < 6; ++i) {
        publish_single(per_joint_position_pubs_[i], positions[i]);
        publish_single(per_joint_velocity_pubs_[i], velocities[i]);
        publish_single(per_joint_torque_pubs_[i],   torques[i]);
        publish_single(per_joint_current_pubs_[i],  currents[i]);
        publish_single(per_joint_temperature_pubs_[i], temperatures[i]);
    }
}

void ArmControllerTestNode::publish_multi(
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub,
    const std::vector<double>& data)
{
    std_msgs::msg::Float64MultiArray m;
    m.data = data;
    pub->publish(m);
}

void ArmControllerTestNode::publish_single(
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub,
    double value)
{
    std_msgs::msg::Float64 m;
    m.data = value;
    pub->publish(m);
}
