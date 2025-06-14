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
Listen up for five minutes because this gets complicated really quick

Step one is going to be installing moveit because its has some dependancies that for some reason cant be installed alone and the only doc they provide gives you how to set it up using their tutorial ( and because we are not bozos we dont want to push their code onto our codebase )

so from the code pull you start by sourcing their examples ( assuming you setup a workspace already at mkdir -p ~/ros2_ws/src )
```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/moveit/moveit2_tutorials

vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos

sudo apt remove ros-$ROS_DISTRO-moveit*

sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

cd ~/ros2_ws
colcon build --mixin release --parallel-workers 1
```

then after that you can start compiling the arm_controller and arm_trajectory_controller

```bash
colcon build --mixin release --packages-select arm_trajectory_controller
colcon build --symlink-install --packages-select arm_controller
source ~/ros2_ws/install/setup.bash
ros2 launch arm_controller arm_launch.py

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
