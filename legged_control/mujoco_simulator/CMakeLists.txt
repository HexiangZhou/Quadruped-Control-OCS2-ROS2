cmake_minimum_required(VERSION 3.8)
project(mujoco_simulator)

set(MUJOCO_INCLUDE_DIRS /home/zhx/Desktop/zhx_legged_ocs2_master/src/mujoco/mujoco-3.2.2/include)
set(MUJOCO_LIBRARIES /home/zhx/Desktop/zhx_legged_ocs2_master/src/mujoco/mujoco-3.2.2/lib/libmujoco.so)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(glfw3 REQUIRED)
find_package(legged_msgs REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
  include
  ${MUJOCO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(mujoco_simulator src/SimulatorNode.cpp src/MujocoSimulation.cpp)
ament_target_dependencies(mujoco_simulator rclcpp std_msgs legged_msgs ocs2_core Boost) #ros2 link library
target_link_libraries(mujoco_simulator ${MUJOCO_LIBRARIES} glfw) #cmake link library

install(TARGETS mujoco_simulator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY models DESTINATION share/${PROJECT_NAME}/)


ament_package()
