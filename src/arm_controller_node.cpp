#include "arm_controller/arm_controller_node.hpp"
#include <sstream>

using namespace std::chrono_literals;

ArmControllerNode::ArmControllerNode()
: Node("arm_controller_node"), arm_()
{
    // All joints array publishers
    positions_pub_    = create_publisher<std_msgs::msg::Float64MultiArray>("kinova_arm/joints/positions", 10);
    velocities_pub_   = create_publisher<std_msgs::msg::Float64MultiArray>("kinova_arm/joints/velocities", 10);
    torques_pub_      = create_publisher<std_msgs::msg::Float64MultiArray>("kinova_arm/joints/torques", 10);
    currents_pub_     = create_publisher<std_msgs::msg::Float64MultiArray>("kinova_arm/joints/currents", 10);
    temperatures_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("kinova_arm/joints/temperatures", 10);

    // Per-joint publishers for all types of data
    for (size_t i = 0; i < 6; ++i)
    {
        std::string joint_num = std::to_string(i+1);
        per_joint_position_pubs_.push_back(create_publisher<std_msgs::msg::Float64>("kinova_arm/joint" + joint_num + "/position", 10));
        per_joint_velocity_pubs_.push_back(create_publisher<std_msgs::msg::Float64>("kinova_arm/joint" + joint_num + "/velocity", 10));
        per_joint_torque_pubs_.push_back(create_publisher<std_msgs::msg::Float64>("kinova_arm/joint" + joint_num + "/torque", 10));
        per_joint_current_pubs_.push_back(create_publisher<std_msgs::msg::Float64>("kinova_arm/joint" + joint_num + "/current", 10));
        per_joint_temperature_pubs_.push_back(create_publisher<std_msgs::msg::Float64>("kinova_arm/joint" + joint_num + "/temperature", 10));
    }

    timer_ = create_wall_timer(25ms, std::bind(&ArmControllerNode::timer_callback, this));
    // Set positions (all)
    set_positions_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "kinova_arm/joints/setPositions",
        10,
        std::bind(&ArmControllerNode::set_positions_callback, this, std::placeholders::_1)
    );
    // Set velocities (all)
    set_velocities_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "kinova_arm/joints/setVelocities",
        10,
        std::bind(&ArmControllerNode::set_velocities_callback, this, std::placeholders::_1)
    );

    // Per-joint positions & velocities
    const int joint_count = 6; // or whatever your DOF is
    for (int i = 0; i < joint_count; ++i) {
        // Per-joint position
        auto pos_sub = this->create_subscription<std_msgs::msg::Float64>(
            "kinova_arm/joint" + std::to_string(i+1) + "/setPosition",
            10,
            [this, i](const std_msgs::msg::Float64::SharedPtr msg) {
                this->set_position_callback(msg, i);
            }
        );
        set_position_subs_.push_back(pos_sub);

        // Per-joint velocity
        auto vel_sub = this->create_subscription<std_msgs::msg::Float64>(
            "kinova_arm/joint" + std::to_string(i+1) + "/setVelocity",
            10,
            [this, i](const std_msgs::msg::Float64::SharedPtr msg) {
                this->set_velocity_callback(msg, i);
            }
        );
        set_velocity_subs_.push_back(vel_sub);
    }
}

// Set all joints position
void ArmControllerNode::set_positions_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    arm_.sendJointsPosition(msg->data); // expects vector<double>
}

// Set single joint position
void ArmControllerNode::set_position_callback(const std_msgs::msg::Float64::SharedPtr msg, int idx) {
    arm_.sendJointPosition(idx, msg->data); // expects idx, double
}

// Set all joints velocity
void ArmControllerNode::set_velocities_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    std::vector<float> velocities(msg->data.begin(), msg->data.end());
    arm_.sendJointsVelocity(velocities);
}

// Set single joint velocity
void ArmControllerNode::set_velocity_callback(const std_msgs::msg::Float64::SharedPtr msg, int idx) {
    arm_.sendJointVelocity(idx, msg->data);
}

void ArmControllerNode::timer_callback()
{
    // Query Kinova arm state
    std::vector<double> positions    = arm_.getJointsPositions();
    std::vector<float>  velocities   = arm_.getJointsVelocities();
    std::vector<double> torques      = arm_.getJointsTorques();
    std::vector<double> currents     = arm_.getJointsCurrents();
    std::vector<float>  temperatures = arm_.getJointsTemperatures();

    // Publish full array topics
    publish_float64_multi_array(positions_pub_, positions);
    publish_float64_multi_array(velocities_pub_, std::vector<double>(velocities.begin(), velocities.end()));
    publish_float64_multi_array(torques_pub_,   torques);
    publish_float64_multi_array(currents_pub_,  currents);
    publish_float64_multi_array(temperatures_pub_, std::vector<double>(temperatures.begin(), temperatures.end()));

    // Publish per-joint topics
    for (size_t i = 0; i < 6; ++i)
    {
        publish_float64(per_joint_position_pubs_[i], positions[i]);
        publish_float64(per_joint_velocity_pubs_[i], velocities[i]);
        publish_float64(per_joint_torque_pubs_[i],   torques[i]);
        publish_float64(per_joint_current_pubs_[i],  currents[i]);
        publish_float64(per_joint_temperature_pubs_[i], temperatures[i]);
    }
}

void ArmControllerNode::publish_float64_multi_array(
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub,
    const std::vector<double>& data)
{
    std_msgs::msg::Float64MultiArray msg;
    msg.data = data;
    pub->publish(msg);
}

void ArmControllerNode::publish_float64(
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub,
    double value)
{
    std_msgs::msg::Float64 msg;
    msg.data = value;
    pub->publish(msg);
}

