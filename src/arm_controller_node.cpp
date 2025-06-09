#include "arm_controller/arm_controller_node.hpp"

#include <thread>
#include <chrono>

// ===== Constructor =====
ArmControllerNode::ArmControllerNode()
    : Node("kinova_arm_controller_node"),
      arm_("Kinova.API.UsbCommandLayerUbuntu.so")
{
    // -- Command topic
    joint_state_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "command_joint_states", 10,
        std::bind(&ArmControllerNode::jointStateCmdCb, this, std::placeholders::_1));

    // -- Feedback publishers
    joint_state_pub_   = this->create_publisher<sensor_msgs::msg::JointState>("arm_joint_states", 10);
    joint_currents_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("arm_joint_currents", 10);
    joint_temps_pub_    = this->create_publisher<std_msgs::msg::Float32MultiArray>("arm_joint_temperatures", 10);
    joint_torques_pub_  = this->create_publisher<std_msgs::msg::Float32MultiArray>("arm_joint_torques", 10);

    // -- Per-joint feedback publishers
    for (size_t i = 0; i < 6; ++i) {
        temp_pubs_.push_back(this->create_publisher<std_msgs::msg::Float32>("arm_joint_" + std::to_string(i) + "_temperature", 10));
        current_pubs_.push_back(this->create_publisher<std_msgs::msg::Float32>("arm_joint_" + std::to_string(i) + "_current", 10));
        torque_pubs_.push_back(this->create_publisher<std_msgs::msg::Float32>("arm_joint_" + std::to_string(i) + "_torque", 10));
    }

    feedback_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ArmControllerNode::publishFeedback, this));

    // -- Action server: FollowJointTrajectory
    traj_action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
        this,
        "follow_joint_trajectory",
        std::bind(&ArmControllerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ArmControllerNode::handleCancel, this, std::placeholders::_1),
        std::bind(&ArmControllerNode::handleAccepted, this, std::placeholders::_1)
    );

    // -- Home service
    home_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "send_home",
        std::bind(&ArmControllerNode::sendHomeService, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Kinova Arm Controller node started.");
}

// ===== Joint State Command (Topic) =====
void ArmControllerNode::jointStateCmdCb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() == 6) {
        std::lock_guard<std::mutex> lock(arm_mutex_);
        arm_.sendJointsPosition(msg->position);
    } else {
        RCLCPP_WARN(this->get_logger(), "Received JointState with %zu positions, expected 6.", msg->position.size());
    }
}

// ===== Feedback Publisher (Timer callback) =====
void ArmControllerNode::publishFeedback() {
    std::lock_guard<std::mutex> lock(arm_mutex_);

    // --- Get values from driver
    auto names     = arm_.getJointNames();
    auto positions = arm_.getJointsPositions();
    auto velocities= arm_.getJointsVelocities();
    auto torques   = arm_.getJointsTorques();
    auto currents  = arm_.getJointsCurrents();
    auto temps     = arm_.getJointsTemperatures();

    // --- Publish joint state
    sensor_msgs::msg::JointState js_msg;
    js_msg.header.stamp = now();
    js_msg.name         = names;
    js_msg.position     = positions;
    js_msg.velocity.assign(velocities.begin(), velocities.end());
    js_msg.effort       = torques;
    joint_state_pub_->publish(js_msg);

    // --- Publish all-joint arrays
    std_msgs::msg::Float32MultiArray cur_msg, temp_msg, torq_msg;
    for (size_t i = 0; i < 6; ++i) {
        cur_msg.data.push_back(static_cast<float>(currents[i]));
        temp_msg.data.push_back(static_cast<float>(temps[i]));
        torq_msg.data.push_back(static_cast<float>(torques[i]));
    }
    joint_currents_pub_->publish(cur_msg);
    joint_temps_pub_->publish(temp_msg);
    joint_torques_pub_->publish(torq_msg);

    // --- Publish per-joint topics
    for (size_t i = 0; i < 6; ++i) {
        std_msgs::msg::Float32 t, c, tq;
        t.data  = static_cast<float>(temps[i]);
        c.data  = static_cast<float>(currents[i]);
        tq.data = static_cast<float>(torques[i]);
        temp_pubs_[i]->publish(t);
        current_pubs_[i]->publish(c);
        torque_pubs_[i]->publish(tq);
    }
}

// ===== Trajectory Action Server =====
rclcpp_action::GoalResponse ArmControllerNode::handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
    // Accept only if right number of joints and at least one point
    if (goal->trajectory.points.empty() || goal->trajectory.joint_names.size() != 6)
        return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArmControllerNode::handleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>>)
{
    // Accept cancelation (no actual preemption in this simple version)
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmControllerNode::handleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle)
{
    std::thread{std::bind(&ArmControllerNode::executeTrajectory, this, goal_handle)}.detach();
}

void ArmControllerNode::executeTrajectory(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<FollowJointTrajectory::Result>();

    rclcpp::Time start = now();
    for (const auto &pt : goal->trajectory.points) {
        if (pt.positions.size() != 6) continue;
        {
            std::lock_guard<std::mutex> lock(arm_mutex_);
            arm_.sendJointsPosition(pt.positions);
        }
        // Feedback (optional)
        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        feedback->joint_names = goal->trajectory.joint_names;
        feedback->desired = pt;
        feedback->actual.positions = arm_.getJointsPositions();
        auto v = arm_.getJointsVelocities();
        feedback->actual.velocities.assign(v.begin(), v.end());
        goal_handle->publish_feedback(feedback);

        // Wait (respect time_from_start)
        auto now_time = now();
        rclcpp::Duration time_to_wait = pt.time_from_start - (now_time - start);
        if (time_to_wait.seconds() > 0)
            rclcpp::sleep_for(std::chrono::duration<double>(time_to_wait.seconds()));
    }
    result->error_code = result->SUCCESSFUL;
    goal_handle->succeed(result);
}

// ===== Service: Send Home =====
void ArmControllerNode::sendHomeService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    std::lock_guard<std::mutex> lock(arm_mutex_);
    bool ok = arm_.sendToHome();
    res->success = ok;
    res->message = ok ? "Sent to home." : "Failed to home arm.";
}
