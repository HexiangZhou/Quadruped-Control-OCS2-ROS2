# Quadruped-Control-OCS2-ROS2
## Introduction
This project is based on the open-source library of ocs2_ros2, setting up a simulation environment based on MuJoCo. It implements user command input, NMPC motion control, and quadruped robot physics simulation within the ROS 2 environment. 
![image](.image/structure.jpg)
![image](.image/mujoco_simulation_image.png)

## Installation
### Prerequisites
* Ubuntu 22.04
* ros2 humble
* C++ compiler with C++17 support
* Eigen (v3.4)
* Boost C++ (v1.74)

### Source code
The legged control project is developed based on the open-source library of [OCS2](https://github.com/leggedrobotics/ocs2.git) and [qiayuanl/legged_control](https://github.com/qiayuanl/legged_control.git)
```
# Clone legged_control
git clone https://github.com/HexiangZhou/Quadruped-Control-OCS2-ROS2.git
```

### OCS2_ROS2
The ocs2_ros2 library is based on [zhengxiang94/ocs2_ros2](https://github.com/zhengxiang94/ocs2_ros2.git)
```
# Clone ocs2_ros2 in ros2_ws/src/Quadruped-Control-OCS2-ROS2
cd ~/ros2_ws/src/Quadruped-Control-OCS2-ROS2
# Clone ocs2_ros2
git clone https://github.com/zhengxiang94/ocs2_ros2.git
# Clone pinocchio
git clone --recurse-submodules https://github.com/zhengxiang94/pinocchio.git
# Clone hpp-fcl
git clone --recurse-submodules https://github.com/zhengxiang94/hpp-fcl.git
# Clone ocs2_robotic_assets
git clone https://github.com/zhengxiang94/ocs2_robotic_assets.git
# Clone plane_segmentation_ros2
git clone https://github.com/zhengxiang94/plane_segmentation_ros2.git
```

### Others
```
# Install dependencies
sudo apt-get install ros-humble-grid-map-cv ros-humble-grid-map-msgs ros-humble-grid-map-ros ros-humble-grid-map-sdf ros-humble-octomap libmpfr-dev libpcap-dev libglpk-dev libglfw3-dev
```

## Build
### Build ocs2
```
# If downloading the dependencies via HTTPS fails, use SSH instead
git config --global url."git@github.com:".insteadOf https://github.com/
colcon build --packages-up-to ocs2_legged_robot_ros ocs2_self_collision_visualization --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
Ensure you can test ANYmal using command below:
```
source install/setup.bash
ros2 launch ocs2_legged_robot_ros legged_robot_sqp.launch.py
```
![](.image/ocs2_gif.gif)
### Build mujoco_simulator

If the MuJoCo software is installed in the ros2_ws/src/Quadruped-Control-OCS2-ROS2 folder, you need to modify the CMakeLists.txt in the mojoco_simulator package as follows:
1. Set the MuJoCo include directory and MuJoCo library in the CMakeLists.txt:
```
set(MUJOCO_INCLUDE_DIRS ~/ros2_ws/src/Quadruped-Control-OCS2-ROS2/mujoco/mujoco-3.2.2/include)  # Replace with your own project absolute path
set(MUJOCO_LIBRARIES ~/ros2_ws/src/Quadruped-Control-OCS2-ROS2/mujoco/mujoco-3.2.2/lib/libmujoco.so)  # Replace with your own project absolute path
```
2. Add the path of libmujoco.so.3.2.2 to the LD_LIBRARY_PATH environment variable.
```
# Open the .bashrc file
gedit ~/.bashrc
# Add the following line to include the MuJoCo key path, library path, and binary directory
export MUJOCO_KEY_PATH=~/ros2_ws/src/Quadruped-Control-OCS2-ROS2/mujoco${MUJOCO_KEY_PATH}  # Replace with your own project absolute path
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/ros2_ws/src/Quadruped-Control-OCS2-ROS2/mujoco/mujoco-3.2.2/bin  # Replace with your own project absolute path
export LD_LIBRARY_PATH=~/ros2_ws/src/Quadruped-Control-OCS2-ROS2/mujoco/mujoco-3.2.2/lib:$LD_LIBRARY_PATH  # Replace with your own project absolute path
# Source the .bashrc file:
source ~/.bashrc
```
3. Build mujoco_simulator
```
colcon build --packages-up-to mujoco_simulator
```
### Build motion_control
If the qpOASES package is installed in the ros2_ws/src/Quadruped-Control-OCS2-ROS2 folder, you need to modify the CMakeLists.txt in the motion_control package as follows:
1. Revise CMakeLists.txt:
```
#Add the qpOASES header directory and library directory
include_directories(~/ros2_ws/src/Quadruped-Control-OCS2-ROS2/qpOASES-master/include)  # Modify the path to match your project
link_directories(~/ros2_ws/src/Quadruped-Control-OCS2-ROS2/qpOASES-master/build/libs)  # Modify the path to match your project
#Explicitly link the libqpOASES.a library in the target_link_libraries section
target_link_libraries(${PROJECT_NAME} ~/ros2_ws/src/Quadruped-Control-OCS2-ROS2/qpOASES-master/build/libs/libqpOASES.a) # Modify the path to match your project
```
2. Build motion_control
```
colcon build --packages-up-to motion_control
```
### Build user_command
```
colcon build --packages-up-to user_command
```
### Build ros2 launch
```
colcon build --packages-up-to launch_simulation
```

## Quick Start
```
source install/local_setup.sh
# robot type: [a1,b1] (unitree a1, unitree b1)
ros2 launch launch_simulation legged_robot_sqp.launch.py robot_type:=b1
```
### Command
You can use the user_command shell to control the gait and movement of the legged robot.

**Examples:**
* Use "gait:trot" to switch the gait to trot.
* Use "goal:1 1 0 0" to set the movement goal to "(x, y, z, yaw)=(1, 1, 0, 0)".

**Demonstration:**
![](.image/simulation_video_gif.gif)

