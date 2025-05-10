cmake_minimum_required(VERSION 3.8)
project(arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)  

# Include directories
include_directories(include)

# Generate service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveJoints.srv"
  DEPENDENCIES std_msgs
)

# Executable node
add_executable(arm_controller_node
  src/main.cpp
  src/arm_controller_node.cpp
  src/KinovaArmController.cpp
)

ament_target_dependencies(arm_controller_node
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
)

# Link generated interfaces to the executable
rosidl_target_interfaces(arm_controller_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install executable
install(TARGETS arm_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
