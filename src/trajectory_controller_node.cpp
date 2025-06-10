#include "arm_controller/trajectory_controller_node.hpp"
#include <chrono>
#include <thread>

TrajectoryControllerNode::TrajectoryControllerNode()
: Node("trajectory_controller_node"), arm_()
{
    traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "joint_trajectory", 10,
        std::bind(&TrajectoryControllerNode::trajectory_callback, this, std::placeholders::_1)
    );
}

void TrajectoryControllerNode::trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    if (msg->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty trajectory");
        return;
    }

    rclcpp::Time start_time = this->get_clock()->now();
    rclcpp::Time last_time = start_time;

    for (const auto &point : msg->points) {
        if (point.positions.size() != 6) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory point does not have 6 positions");
            return;
        }

        std::vector<double> positions(point.positions.begin(), point.positions.end());
        arm_.sendJointsPosition(positions);

        // Respect timing if time_from_start is provided
        if (point.time_from_start.sec != 0 || point.time_from_start.nanosec != 0) {
            rclcpp::Time target_time = start_time + point.time_from_start;
            auto now = this->get_clock()->now();
            if (target_time > now) {
                auto sleep_duration = target_time - now;
                std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_duration.nanoseconds()));
            }
            last_time = target_time;
        } else {
            // small delay between points if no timing
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

