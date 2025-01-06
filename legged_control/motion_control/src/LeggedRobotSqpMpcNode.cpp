/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/


#include "motion_control/legged_interface/LeggedRobotInterface.h"//#include <ocs2_legged_robot/LeggedRobotInterface.h>

#include "motion_control/ros_interfaces/RosReferenceManager.h"// #include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
// #include <ocs2_ros_interfaces/synchronized_module/SolverObserverRosCallbacks.h>
#include <ocs2_sqp/SqpMpc.h>

#include "motion_control/ros_interfaces/GaitReceiver.h"//#include "ocs2_legged_robot_ros/gait/GaitReceiver.h"

#include "motion_control/ros_interfaces/MPC_WBC_ROS_Interface.h"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  const std::string robotName = "legged_robot";

  // Initialize ros node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
      robotName + "_mpc",
      rclcpp::NodeOptions()
          .allow_undeclared_parameters(true)
          .automatically_declare_parameters_from_overrides(true));
  // Get node parameters
  bool multiplot = false;
  const std::string taskFile = node->get_parameter("taskFile").as_string();
  const std::string urdfFile = node->get_parameter("urdfFile").as_string();
  const std::string referenceFile = node->get_parameter("referenceFile").as_string();
  const std::string simulatorFile = node->get_parameter("simulatorFile").as_string();
  // const std::string taskFile = "/home/zhx/Desktop/zhx_legged_ocs2_master/src/legged_control/user_command/config/b1/task.info";
  // const std::string urdfFile = "/home/zhx/Desktop/zhx_legged_ocs2_master/src/legged_control/mujoco_simulator/models/b1/urdf/robot.urdf";
  // const std::string referenceFile ="/home/zhx/Desktop/zhx_legged_ocs2_master/src/legged_control/user_command/config/b1/reference.info";
  // const std::string simulatorFile = "/home/zhx/Desktop/zhx_legged_ocs2_master/src/legged_control/user_command/config/b1/simulation.info";

  MPC_WBC_ROS_Interface mpcNode(node, taskFile, urdfFile, referenceFile, simulatorFile, robotName);
  mpcNode.launchNodes();

  // Successful exit
  std::cerr << "Succeed " << std::endl;
  return 0;
}
