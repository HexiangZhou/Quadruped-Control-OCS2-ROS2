cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)
project(motion_control)

set(dependencies
  rclcpp
  tf2_ros
  urdf
  kdl_parser
  robot_state_publisher
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_ipm
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_robotic_assets
  legged_msgs
  #ocs2_msgs
  #ocs2_ros_interfaces
  #ocs2_legged_robot
  Boost
  angles
  nav_msgs
)

find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_ipm REQUIRED)
#find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(legged_msgs REQUIRED)
#find_package(ocs2_msgs REQUIRED)
#find_package(ocs2_legged_robot REQUIRED)
find_package(rclcpp REQUIRED)
find_package(angles REQUIRED)
find_package(nav_msgs REQUIRED)



find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Manually add qpOASES (compiled from source)
include_directories(/home/zhx/Desktop/zhx_legged_ocs2_master/src/qpOASES-master/include)
link_directories(/home/zhx/Desktop/zhx_legged_ocs2_master/src/qpOASES-master/build/libs)

# Add directories for all targets
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

###########
## Build ##
###########

# Main library
add_library(${PROJECT_NAME}
  src/common/ModelSettings.cpp
  src/gait/ModeSequenceTemplate.cpp
  src/gait/GaitSchedule.cpp
  src/gait/Gait.cpp
  src/legged_interface/LeggedRobotInterface.cpp
  src/reference_manager/SwitchedModelReferenceManager.cpp
  src/foot_planner/SwingTrajectoryPlanner.cpp
  src/foot_planner/CubicSpline.cpp
  src/foot_planner/SplineCpg.cpp
  src/initialization/LeggedRobotInitializer.cpp
  src/constraint/EndEffectorLinearConstraint.cpp
  src/constraint/FrictionConeConstraint.cpp
  src/constraint/NormalVelocityConstraintCppAd.cpp
  src/constraint/ZeroForceConstraint.cpp
  src/constraint/ZeroVelocityConstraintCppAd.cpp
  src/robot_precomputation/LeggedRobotPreComputation.cpp
  src/dynamics/LeggedRobotDynamicsAD.cpp
  src/ros_interfaces/GaitReceiver.cpp
  src/ros_interfaces/RosReferenceManager.cpp
  src/ros_interfaces/MPC_WBC_ROS_Interface.cpp
  src/legged_wbc/WbcBase.cpp
  src/legged_wbc/WeightedWbc.cpp
  src/legged_wbc/HoQp.cpp
  src/legged_wbc/HierarchicalWbc.cpp
  src/legged_estimation/StateEstimateBase.cpp
  src/legged_estimation/LinearKalmanFilter.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME} /home/zhx/Desktop/zhx_legged_ocs2_master/src/qpOASES-master/build/libs/libqpOASES.a)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

## SQP-MPC node for legged robot
add_executable(legged_robot_sqp_mpc
  src/LeggedRobotSqpMpcNode.cpp
)
ament_target_dependencies(legged_robot_sqp_mpc
  ${dependencies}
)
target_link_libraries(legged_robot_sqp_mpc
  ${PROJECT_NAME}
  /home/zhx/Desktop/zhx_legged_ocs2_master/src/qpOASES-master/build/libs/libqpOASES.a
)
target_compile_options(legged_robot_sqp_mpc  PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
   message(STATUS "Run clang tooling for target ocs2_legged_robot_ros")
   add_clang_tooling(
     TARGETS
       ${PROJECT_NAME}
       legged_robot_sqp_mpc
     SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
     CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
     CF_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS
    legged_robot_sqp_mpc
    DESTINATION lib/${PROJECT_NAME}
)
#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
#install(DIRECTORY launch rviz
  #DESTINATION share/${PROJECT_NAME}/
#)

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
