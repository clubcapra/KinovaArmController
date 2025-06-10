<a name="readme-top"></a>
<!-- PROJECT LOGO -->
<br />
<div align="center">
  <h3 align="center">KinovaArmController</h3>
  <p align="center">
    A ROS2 Node for controlling a Kinova 6 axis arm
    <br />
    <a href="https://github.com/clubcapra/KinovaArmController"><strong>Explore the Documentation »</strong></a>
    <br />
  </p>
</div>

<!-- ABOUT THE PROJECT -->
## About this Project
This project is a ROS2 Node for controlling a Kinova 6 axis arm, it contains the node and a KinovaApi Wrapper.

<!-- TECHSTACK -->
## Tech Stack :
[![CPP][CPP.com]][CPP-url]
[![ROS2Humble][ROS2Humble.com]][ROS2Humble-url]

<!-- SETUP -->
## Setup
### Install Kinova Tools
Download and install the Kinova SDK and GUI Tool 
- Go to : https://www.kinovarobotics.com/resources
- Search for the Gen2 SDK v1.5.1

### Install ROS2
- No Comments

### Source and Run
```bash
colcon build --symlink-install --packages-select arm_controller
source ~/ros2_ws/install/setup.bash
ros2 launch arm_controller arm_launch.py
```

### Trajectory Controller
To execute trajectories generated from an inverse kinematics solver, run the new
`trajectory_controller_node`:

```bash
ros2 run arm_controller trajectory_controller_node
```

Send a `JointTrajectory` message containing the desired joint positions and
timing. For example:

```bash
ros2 topic pub /joint_trajectory trajectory_msgs/msg/JointTrajectory "{\n  joint_names: ['joint1','joint2','joint3','joint4','joint5','joint6'],\n  points: [{positions: [0,0,0,0,0,0], time_from_start: {sec: 1}}, {positions: [1.0,0,0,0,0,0], time_from_start: {sec: 3}}]\n}"
```

Sends a velocity per seconds of 30 to the actuator0 of the arm
```bash
ros2 topic pub /joint_velocities std_msgs/msg/Float64MultiArray "{data: [30,0,0,0,0,0]}"
```

<!-- ANNEX -->
## Annex
### Arm
Spec sheet: kinova_gen2_spherical_spec_sheet.pdf
User Guide: kinova_gen2_user_guide.pdf

### Actuators
Spec sheet: kinova_actuator_series_ka75+_ka-58_specifications.pdf
User Guide: kinova_actuator_series_user_guide.pdf

### Controller
Spec sheet: kinova_controller_specifications.pdf

<!-- LICENSE -->
## License
Distributed under proprietary license. See `LICENSE.md` for more informations.

<!-- MARKDOWN LINKS & IMAGES -->
[CPP.com]: https://img.shields.io/badge/-C++-blue?logo=cplusplus
[CPP-url]: https://www.open-std.org/jtc1/sc22/wg21/

[ROS2Humble.com]: https://img.shields.io/badge/ROS2-Humble-brightgreen.svg
[ROS2Humble-url]: https://docs.ros.org/en/humble/index.html
