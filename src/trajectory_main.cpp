#include "rclcpp/rclcpp.hpp"
#include "arm_controller/trajectory_controller_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryControllerNode>());
  rclcpp::shutdown();
  return 0;
}
