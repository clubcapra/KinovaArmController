#include "rclcpp/rclcpp.hpp"
#include "arm_controller/arm_controller_test_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmControllerTestNode>());
  rclcpp::shutdown();
  return 0;
}