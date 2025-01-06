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

//#include "ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h"
#include "user_command/goal/TargetTrajectoriesKeyboardPublisher.h"

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/Display.h>
// #include <ocs2_ros_interfaces/common/RosMsgConversions.h>



/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectoriesKeyboardPublisher::TargetTrajectoriesKeyboardPublisher(
    const rclcpp::Node::SharedPtr& node, const std::string& topicPrefix,
    const ocs2::scalar_array_t& targetCommandLimits,
    //CommandLineToTargetTrajectories commandLineToTargetTrajectoriesFun,
    const std::string& referenceFile)
    : node_(node),
      targetCommandLimits_(Eigen::Map<const ocs2::vector_t>(
          targetCommandLimits.data(), targetCommandLimits.size())),
      // commandLineToTargetTrajectoriesFun_(
      //     std::move(commandLineToTargetTrajectoriesFun)),
          defaultJointState_(12) {
  ocs2::loadData::loadCppDataType(referenceFile, "comHeight", comHeight_);
  ocs2::loadData::loadEigenMatrix(referenceFile, "defaultJointState",
                            defaultJointState_);
  ocs2::loadData::loadCppDataType(referenceFile, "targetRotationVelocity",
                            targetRotationVelocity_);
  ocs2::loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity",
                            targetDisplacementVelocity_);
  
  // observation subscriber
  auto observationCallback =
      [this](const legged_msgs::msg::MpcObservation::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(latestObservationMutex_); 
        //By locking latestObservationMutex_, it ensures that the following code block is thread-safe, 
        //preventing multiple threads from simultaneously accessing or modifying the shared resource latestObservation_.
        latestObservation_ = readObservationMsg(*msg);
        std::cout << "observation recieved" << std::endl;
      };
  observationSubscriber_ =
      node->create_subscription<legged_msgs::msg::MpcObservation>(
          topicPrefix + "_mpc_observation", 1, observationCallback);

  // // Trajectories publisher
  // targetTrajectoriesPublisherPtr_.reset(
  //     new TargetTrajectoriesRosPublisher(node, topicPrefix));
  targetTrajectoriesPublisherPtr_=
      node_->create_publisher<legged_msgs::msg::MpcTargetTrajectories>(topicPrefix + "_mpc_target", 1); 

  // Initialize latestObservation_
  latestObservation_.state = Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1>::Zero(24);
  latestObservation_.input = Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1>::Zero(24);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesKeyboardPublisher::publishKeyboardCommand(std::string& commandValue) {
    // a line to words
  const std::vector<std::string> words = ocs2::stringToWords(commandValue);

  const size_t targetCommandSize = targetCommandLimits_.size();
  ocs2::vector_t targetCommand = ocs2::vector_t::Zero(targetCommandSize);
  for (size_t i = 0; i < std::min(words.size(), targetCommandSize); i++) {
    targetCommand(i) = static_cast<ocs2::scalar_t>(stof(words[i]));
  }
  const ocs2::vector_t commandLineInput = targetCommand
                                        .cwiseMin(targetCommandLimits_)
                                        .cwiseMax(-targetCommandLimits_);

  // display
  std::cout << "The following command is publishing: ["
            << ocs2::toDelimitedString(commandLineInput) << "]\n";

  // get the latest observation
  rclcpp::spin_some(node_->get_node_base_interface());
  ocs2::SystemObservation observation;
  {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    observation = latestObservation_;
  }
  
  // get TargetTrajectories
  const auto targetTrajectories =
      commandLineToTargetTrajectories(commandLineInput, observation);

  // publish TargetTrajectories
  const auto mpcTargetTrajectoriesMsg = createTargetTrajectoriesMsg(targetTrajectories);
  targetTrajectoriesPublisherPtr_->publish(mpcTargetTrajectoriesMsg);
  std::cout << "Goal publish succeed!!!.\n";
  std::cout << std::endl;
}

// /******************************************************************************************************/
// /******************************************************************************************************/
// /******************************************************************************************************/
// void TargetTrajectoriesKeyboardPublisher::publishKeyboardCommand(
//     const std::string& commadMsg) {
//   while (rclcpp::ok()) {
//     // get command line
//     std::cout << commadMsg << ": ";
//     const ocs2::vector_t commandLineInput = getCommandLine()
//                                           .cwiseMin(targetCommandLimits_)
//                                           .cwiseMax(-targetCommandLimits_); //.cwiseMin and .cwiseMax are from Eigen library

//     // display
//     std::cout << "The following command is published: ["
//               << ocs2::toDelimitedString(commandLineInput) << "]\n\n";

//     std::cout << "success" << ": ";

//     // get the latest observation
//     rclcpp::spin_some(node_->get_node_base_interface());
//     ocs2::SystemObservation observation;
//     {
//       std::lock_guard<std::mutex> lock(latestObservationMutex_);
//       observation = latestObservation_;
//     }
    
//     // get TargetTrajectories
//     const auto targetTrajectories =
//         commandLineToTargetTrajectories(commandLineInput, observation);

//     // publish TargetTrajectories
//     const auto mpcTargetTrajectoriesMsg = createTargetTrajectoriesMsg(targetTrajectories);
//     targetTrajectoriesPublisherPtr_->publish(mpcTargetTrajectoriesMsg);
//   }  // end of while loop
// }


// /******************************************************************************************************/
// /******************************************************************************************************/
// /******************************************************************************************************/
// ocs2::vector_t TargetTrajectoriesKeyboardPublisher::getCommandLine() {
//   // get command line as one long string
//   auto shouldTerminate = []() { return !rclcpp::ok(); };
//   const std::string line = ocs2::getCommandLineString(shouldTerminate);
//   // could add if else here to decide gait and goal

//   // a line to words
//   const std::vector<std::string> words = ocs2::stringToWords(line);

//   const size_t targetCommandSize = targetCommandLimits_.size();
//   ocs2::vector_t targetCommand = ocs2::vector_t::Zero(targetCommandSize);
//   for (size_t i = 0; i < std::min(words.size(), targetCommandSize); i++) {
//     targetCommand(i) = static_cast<ocs2::scalar_t>(stof(words[i]));
//   }

//   return targetCommand;
// }


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// transfer legged_msgs::msg::MpcObservation to ocs2::SystemObservation
// the former contains float array, the later contains Eigen::Matrix. The values are same.
ocs2::SystemObservation TargetTrajectoriesKeyboardPublisher::readObservationMsg(
    const legged_msgs::msg::MpcObservation& observationMsg) {
  ocs2::SystemObservation observation;

  observation.time = observationMsg.time;

  const auto& state = observationMsg.state.value;
  observation.state =
      Eigen::Map<const Eigen::VectorXf>(state.data(), state.size())
          .cast<ocs2::scalar_t>();

  const auto& input = observationMsg.input.value;
  observation.input =
      Eigen::Map<const Eigen::VectorXf>(input.data(), input.size())
          .cast<ocs2::scalar_t>();

  observation.mode = observationMsg.mode;

  return observation;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
legged_msgs::msg::MpcTargetTrajectories TargetTrajectoriesKeyboardPublisher::createTargetTrajectoriesMsg(
    const ocs2::TargetTrajectories& targetTrajectories) {
  legged_msgs::msg::MpcTargetTrajectories targetTrajectoriesMsg;
  const auto& timeTrajectory = targetTrajectories.timeTrajectory;
  const auto& stateTrajectory = targetTrajectories.stateTrajectory;
  const auto& inputTrajectory = targetTrajectories.inputTrajectory;

  // time and state
  size_t N = stateTrajectory.size();
  targetTrajectoriesMsg.time_trajectory.resize(N);
  targetTrajectoriesMsg.state_trajectory.resize(N);
  for (size_t i = 0; i < N; i++) {
    targetTrajectoriesMsg.time_trajectory[i] = timeTrajectory[i];

    targetTrajectoriesMsg.state_trajectory[i].value = std::vector<float>(
        stateTrajectory[i].data(),
        stateTrajectory[i].data() + stateTrajectory[i].size());
  }  // end of i loop

  // input
  N = inputTrajectory.size();
  targetTrajectoriesMsg.input_trajectory.resize(N);
  for (size_t i = 0; i < N; i++) {
    targetTrajectoriesMsg.input_trajectory[i].value = std::vector<float>(
        inputTrajectory[i].data(),
        inputTrajectory[i].data() + inputTrajectory[i].size());
  }  // end of i loop

  return targetTrajectoriesMsg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2::scalar_t TargetTrajectoriesKeyboardPublisher::estimateTimeToTarget(const ocs2::vector_t& desiredBaseDisplacement) {
  const ocs2::scalar_t& dx = desiredBaseDisplacement(0);
  const ocs2::scalar_t& dy = desiredBaseDisplacement(1);
  const ocs2::scalar_t& dyaw = desiredBaseDisplacement(3);
  const ocs2::scalar_t rotationTime = std::abs(dyaw) / targetRotationVelocity_;
  const ocs2::scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const ocs2::scalar_t displacementTime = displacement / targetDisplacementVelocity_;
  return std::max(rotationTime, displacementTime);
}

/**
 * Converts command line to TargetTrajectories.
 * @param [in] commadLineTarget : [deltaX, deltaY, deltaZ, deltaYaw]
 * @param [in] observation : the current observation
 */
ocs2::TargetTrajectories TargetTrajectoriesKeyboardPublisher::commandLineToTargetTrajectories(
    const ocs2::vector_t& commadLineTarget, const ocs2::SystemObservation& observation) {
      
  const ocs2::vector_t currentPose = observation.state.segment<6>(6);
  const ocs2::vector_t targetPose = [&]() {
    ocs2::vector_t target(6);
    // base p_x, p_y are relative to current state
    target(0) = currentPose(0) + commadLineTarget(0);
    target(1) = currentPose(1) + commadLineTarget(1);
    // base z relative to the default height
    target(2) = comHeight_ + commadLineTarget(2);
    // theta_z relative to current
    target(3) = currentPose(3) + commadLineTarget(3) * M_PI / 180.0;
    // theta_y, theta_x
    target(4) = currentPose(4);
    target(5) = currentPose(5);
    return target;
  }();
  
  // target reaching duration
  const ocs2::scalar_t targetReachingTime =
      observation.time + estimateTimeToTarget(targetPose - currentPose);

  // desired time trajectory
  const ocs2::scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // desired state trajectory
  ocs2::vector_array_t stateTrajectory(2, ocs2::vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << ocs2::vector_t::Zero(6), currentPose, defaultJointState_;
  stateTrajectory[1] << ocs2::vector_t::Zero(6), targetPose, defaultJointState_;

  // desired input trajectory (just right dimensions, they are not used)
  const ocs2::vector_array_t inputTrajectory(
      2, ocs2::vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}