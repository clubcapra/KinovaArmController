#include "arm_controller/arm_controller_node.hpp"
#include <sstream>

ArmControllerNode::ArmControllerNode()
: Node("arm_controller_node"), arm_()
{
  position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "current_joint_positions", 10);

  velocity_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "joint_velocities", 10,
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      if (msg->data.size() != 6) {
        RCLCPP_WARN(this->get_logger(),
          "Expected 6 velocities, got %zu", msg->data.size());
        return;
      }

      bool ok = arm_.sendJointVelocity(msg->data);
      if (!ok) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send joint velocities");
        return;
      }

      std::vector<double> positions = arm_.getCurrentJointPositions();

      std_msgs::msg::Float64MultiArray pos_msg;
      pos_msg.data = positions;
      position_pub_->publish(pos_msg);

      std::ostringstream ss;
      ss << "Current joint positions: [";
      for (size_t i = 0; i < positions.size(); ++i) {
        ss << positions[i] << (i + 1 < positions.size() ? ", " : "");
      }
      ss << "]";
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    });
}
