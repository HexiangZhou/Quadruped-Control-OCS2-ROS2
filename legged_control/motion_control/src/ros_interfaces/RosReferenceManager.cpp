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

#include "motion_control/ros_interfaces/RosReferenceManager.h"//#include "ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h"

//#include "ocs2_ros_interfaces/common/RosMsgConversions.h"
#include "rclcpp/rclcpp.hpp"

// MPC messages
#include "legged_msgs/msg/gait_mode_schedule.hpp"
#include "legged_msgs/msg/mpc_target_trajectories.hpp"


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RosReferenceManager::RosReferenceManager(
    std::string topicPrefix,
    std::shared_ptr<ocs2::ReferenceManagerInterface> referenceManagerPtr)
    : ocs2::ReferenceManagerDecorator(std::move(referenceManagerPtr)),
      topicPrefix_(std::move(topicPrefix)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RosReferenceManager::subscribe(const rclcpp::Node::SharedPtr& node) {
  node_ = node;
  // ModeSchedule
  auto modeScheduleCallback = [this](const legged_msgs::msg::GaitModeSchedule& msg) {
    auto modeSchedule = readModeScheduleMsg(msg);
    referenceManagerPtr_->setModeSchedule(std::move(modeSchedule));
  };
  modeScheduleSubscriber_ =
      node_->create_subscription<legged_msgs::msg::GaitModeSchedule>(
          topicPrefix_ + "_mode_schedule", 1, modeScheduleCallback);

  // TargetTrajectories
  auto targetTrajectoriesCallback =
      [this](const legged_msgs::msg::MpcTargetTrajectories& msg) {
        auto targetTrajectories =
            readTargetTrajectoriesMsg(msg);
        referenceManagerPtr_->setTargetTrajectories(
            std::move(targetTrajectories));
        std::cout << "goal recieved!" << std::endl;
      };
  targetTrajectoriesSubscriber_ =
      node_->create_subscription<legged_msgs::msg::MpcTargetTrajectories>(
          topicPrefix_ + "_mpc_target", 1, targetTrajectoriesCallback);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2::ModeSchedule RosReferenceManager::readModeScheduleMsg(
    const legged_msgs::msg::GaitModeSchedule& modeScheduleMsg) {
  // event times
  ocs2::scalar_array_t eventTimes;
  eventTimes.reserve(modeScheduleMsg.event_times.size());
  for (const auto& ti : modeScheduleMsg.event_times) {
    eventTimes.push_back(ti);
  }

  // mode sequence
  ocs2::size_array_t mode_sequence;
  mode_sequence.reserve(modeScheduleMsg.mode_sequence.size());
  for (const auto& si : modeScheduleMsg.mode_sequence) {
    mode_sequence.push_back(si);
  }

  return {eventTimes, mode_sequence};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2::TargetTrajectories RosReferenceManager::readTargetTrajectoriesMsg(
    const legged_msgs::msg::MpcTargetTrajectories& targetTrajectoriesMsg) {
  size_t N = targetTrajectoriesMsg.state_trajectory.size();
  if (N == 0) {
    throw std::runtime_error(
        "An empty target trajectories message is received.");
  }

  // state and time
  ocs2::scalar_array_t desiredTimeTrajectory(N);
  ocs2::vector_array_t desiredStateTrajectory(N);
  for (size_t i = 0; i < N; i++) {
    desiredTimeTrajectory[i] = targetTrajectoriesMsg.time_trajectory[i];

    desiredStateTrajectory[i] =
        Eigen::Map<const Eigen::VectorXf>(
            targetTrajectoriesMsg.state_trajectory[i].value.data(),
            targetTrajectoriesMsg.state_trajectory[i].value.size())
            .cast<ocs2::scalar_t>();
  }  // end of i loop

  // input
  N = targetTrajectoriesMsg.input_trajectory.size();
  ocs2::vector_array_t desiredInputTrajectory(N);
  for (size_t i = 0; i < N; i++) {
    desiredInputTrajectory[i] =
        Eigen::Map<const Eigen::VectorXf>(
            targetTrajectoriesMsg.input_trajectory[i].value.data(),
            targetTrajectoriesMsg.input_trajectory[i].value.size())
            .cast<ocs2::scalar_t>();
  }  // end of i loop

  return {desiredTimeTrajectory, desiredStateTrajectory,
          desiredInputTrajectory};
}
