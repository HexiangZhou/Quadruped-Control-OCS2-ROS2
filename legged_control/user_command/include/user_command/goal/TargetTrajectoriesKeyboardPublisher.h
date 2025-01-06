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

#include <ocs2_mpc/SystemObservation.h>

#include "legged_msgs/msg/mpc_target_trajectories.hpp"
#include "legged_msgs/msg/mpc_observation.hpp"
#include <ocs2_core/Types.h>
//#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_core/misc/LoadData.h>

#include <functional>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"


/**
 * This class lets the user to insert robot command form command line.
 */
class TargetTrajectoriesKeyboardPublisher {
 public:
  // using CommandLineToTargetTrajectories = std::function<ocs2::TargetTrajectories(
  //     const ocs2::vector_t& commadLineTarget, const ocs2::SystemObservation& observation)>;

  /**
   * Constructor
   *
   * @param [in] node: ROS node handle.
   * @param [in] topicPrefix: The TargetTrajectories will be published on
   * "topicPrefix_mpc_target" topic. Moreover, the latest observation is be
   * expected on "topicPrefix_mpc_observation" topic.
   * @param [in] targetCommandLimits: The limits of the loaded command from
   * command-line (for safety purposes).
   * @param [in] commandLineToTargetTrajectoriesFun: A function which transforms
   * the command line input to TargetTrajectories.
   */
  TargetTrajectoriesKeyboardPublisher(
      const rclcpp::Node::SharedPtr& node, const std::string& topicPrefix,
      const ocs2::scalar_array_t& targetCommandLimits,
      //CommandLineToTargetTrajectories commandLineToTargetTrajectoriesFun,
      const std::string& referenceFile);

  /** Gets the command vector size. */
  size_t targetCommandSize() const { return targetCommandLimits_.size(); }

  /**
   * Publishes command line input. If the input command is shorter than the
   * expected command size (targetCommandSize), the method will set the rest of
   * the command to zero.
   *
   * @param [in] commadMsg: Message to be displayed on screen.
   */
  // void publishKeyboardCommand(
  //     const std::string& commadMsg = "Enter command, separated by space");
  void publishKeyboardCommand(std::string& commandValue);

 private:
  /** Gets the target from command line. */
  //ocs2::vector_t getCommandLine();

  rclcpp::Node::SharedPtr node_;
  const ocs2::vector_t targetCommandLimits_;
  //CommandLineToTargetTrajectories commandLineToTargetTrajectoriesFun_;

  ocs2::SystemObservation readObservationMsg(
    const legged_msgs::msg::MpcObservation& observationMsg);
  legged_msgs::msg::MpcTargetTrajectories createTargetTrajectoriesMsg(
    const ocs2::TargetTrajectories& targetTrajectories);
  ocs2::TargetTrajectories commandLineToTargetTrajectories(
    const ocs2::vector_t& commadLineTarget, const ocs2::SystemObservation& observation);
  ocs2::scalar_t estimateTimeToTarget(const ocs2::vector_t& desiredBaseDisplacement);

  // std::unique_ptr<TargetTrajectoriesRosPublisher>
  //     targetTrajectoriesPublisherPtr_;
  rclcpp::Publisher<legged_msgs::msg::MpcTargetTrajectories>::SharedPtr
      targetTrajectoriesPublisherPtr_;

  rclcpp::Subscription<legged_msgs::msg::MpcObservation>::SharedPtr
      observationSubscriber_;
  mutable std::mutex latestObservationMutex_;
  ocs2::SystemObservation latestObservation_;

  ocs2::scalar_t targetDisplacementVelocity_;
  ocs2::scalar_t targetRotationVelocity_;
  ocs2::scalar_t comHeight_;
  ocs2::vector_t defaultJointState_;
  
};

