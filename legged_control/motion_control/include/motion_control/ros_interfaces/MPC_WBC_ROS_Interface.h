/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#pragma once

// #include <ocs2_core/control/FeedforwardController.h>
// #include <ocs2_core/control/LinearController.h>
// #include <ocs2_core/misc/Benchmark.h>
// #include <ocs2_mpc/CommandData.h>
// #include <ocs2_mpc/MPC_BASE.h>
// #include <ocs2_mpc/SystemObservation.h>
// #include <ocs2_oc/oc_data/PrimalSolution.h>

// #include <atomic>
// #include <condition_variable>
// #include <iostream>
// #include <memory>
// #include <mutex>
// #include <ocs2_msgs/msg/mode_schedule.hpp>
// #include <ocs2_msgs/msg/mpc_flattened_controller.hpp>
// #include <ocs2_msgs/msg/mpc_observation.hpp>
// #include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
// #include <ocs2_msgs/srv/reset.hpp>
// #include <string>
// #include <thread>
// #include <vector>

#include "motion_control/legged_interface/LeggedRobotInterface.h"
#include "motion_control/legged_wbc/WbcBase.h"
#include "motion_control/common/Types.h"
#include "motion_control/gait/MotionPhaseDefinition.h"
#include "motion_control/legged_estimation/StateEstimateBase.h"

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_core/misc/LoadData.h>



#include "legged_msgs/msg/mpc_observation.hpp"
#include "legged_msgs/msg/simulator_state_data.hpp"
#include "legged_msgs/msg/simulator_sensor_data.hpp"
#include "legged_msgs/msg/joint_control_data.hpp"
#include "legged_msgs/srv/start_control.hpp"

#include "rclcpp/rclcpp.hpp"


/**
 * This class implements MPC communication interface using ROS.
 */
class MPC_WBC_ROS_Interface {
public:
  MPC_WBC_ROS_Interface(
    const rclcpp::Node::SharedPtr& node, 
    const std::string& urdfFile,
    const std::string& taskFile,
    const std::string& referenceFile,
    const std::string& simulatorFile,
    const std::string& robotName);

  void launchNodes();

protected:
    void setupMpc(const std::string& robotName);
    void setupMrt();
    void setupWbc(const std::string& taskFile, bool verbose);
    void setupStateEstimate(const std::string& taskFile, bool verbose);
    void simulatorStartControlLoop();
    void setInitialState(const legged_msgs::msg::SimulatorStateData::ConstSharedPtr msg);
    void simulatorStateCallback(const legged_msgs::msg::SimulatorStateData::ConstSharedPtr msg);
    void updateStateEstimationFromState(const legged_msgs::msg::SimulatorStateData::ConstSharedPtr msg);
    void simulatorSensorCallback(const legged_msgs::msg::SimulatorSensorData::ConstSharedPtr msg);
    void updateStateEstimationFromSensor(const legged_msgs::msg::SimulatorSensorData::ConstSharedPtr msg);
    void publishJointControl(const ocs2::vector_t& torque, const ocs2::vector_t& posDes, const ocs2::vector_t& velDes);
    void publishCurrentObservation();
    legged_msgs::msg::MpcObservation createObservationMsg(const ocs2::SystemObservation& observation);
    std::string eigenToString(const ocs2::vector_t& vec);
    
    std::string robotName_;
    // Interface
    std::shared_ptr<LeggedRobotInterface> leggedInterface_;

    // Nonlinear MPC
    int Mpc_control_frequency_;
    std::shared_ptr<ocs2::MPC_BASE> mpc_;
    std::shared_ptr<ocs2::MPC_MRT_Interface> mpcMrtInterface_;

    // Whole Body Control
    int Wbc_control_frequency_;
    std::shared_ptr<ocs2::PinocchioEndEffectorKinematics> eeKinematicsPtr_;
    std::shared_ptr<WbcBase> wbc_;

    // State Estimation
    ocs2::SystemObservation currentObservation_;
    ocs2::vector_t measuredRbdState_;
    std::shared_ptr<StateEstimateBase> stateEstimate_;
    std::shared_ptr<ocs2::CentroidalModelRbdConversions> rbdConversions_;
    contact_flag_t contactFlag_{};

    // ROS
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<legged_msgs::msg::MpcObservation>::SharedPtr observationPublisher_;
    rclcpp::Publisher<legged_msgs::msg::JointControlData>::SharedPtr jointControlPublisher_;
    rclcpp::Subscription<legged_msgs::msg::SimulatorStateData>::SharedPtr simulatorStateSubscriber_;
    rclcpp::Subscription<legged_msgs::msg::SimulatorSensorData>::SharedPtr simulatorSensorSubscriber_;
    rclcpp::Client<legged_msgs::srv::StartControl>::SharedPtr controlStartingClient_; 


private:
    ocs2::benchmark::RepeatedTimer mpcTimer_; // mpcMrtInterface_ also has a mpcTimer_. but mpcTimer_ is more comprehensive
    ocs2::benchmark::RepeatedTimer wbcTimer_;
    int MpcCount_; // using count to control different frequency of mpc and wbc
    bool StateEstimate_; //use state estimate (true) or use real state from simulator (false)
    

  /*
   * Variables
   */

//   std::string topicPrefix_;

//   rclcpp::Node::SharedPtr node_;

//   // Publishers and subscribers
//   rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr
//       mpcObservationSubscriber_;
//   rclcpp::Subscription<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr
//       mpcTargetTrajectoriesSubscriber_;
//   rclcpp::Publisher<ocs2_msgs::msg::MpcFlattenedController>::SharedPtr
//       mpcPolicyPublisher_;
//   rclcpp::Service<ocs2_msgs::srv::Reset>::SharedPtr mpcResetServiceServer_;
};



// // convert quat to eulerAngles
// template <typename T>
// T square(T a) {
//   return a * a;
// }

// template <typename SCALAR_T>
// Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T>& q) {
//     Eigen::Matrix<SCALAR_T, 3, 1> zyx;

//     SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
//     zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
//     zyx(1) = std::asin(as);
//     zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
//     return zyx;
// }

