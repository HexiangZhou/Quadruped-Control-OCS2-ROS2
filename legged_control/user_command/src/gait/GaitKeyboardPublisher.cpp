/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include "user_command/gait/GaitKeyboardPublisher.h"

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/LoadData.h>

#include <algorithm>
#include "legged_msgs/msg/gait_mode_schedule.hpp"
//#include <ocs2_msgs/msg/mode_schedule.hpp>

//#include "ocs2_legged_robot_ros/gait/ModeSequenceTemplateRos.h"


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaitKeyboardPublisher::GaitKeyboardPublisher(
    const rclcpp::Node::SharedPtr& node, const std::string& gaitFile,
    const std::string& robotName, bool verbose) {
  RCLCPP_INFO_STREAM(node->get_logger(),
                     robotName + "_mpc_mode_schedule node is setting up ...");

  // load gaitlist from gait.info using ocs2 lib
  ocs2::loadData::loadStdVector(gaitFile, "list", gaitList_, verbose);

  modeSequenceTemplatePublisher_ =
      node->create_publisher<legged_msgs::msg::GaitModeSchedule>(
          robotName + "_mpc_mode_schedule", 1);

  //gaitMap_ contains all the gait modeSequence and switchingTimes vectors
  gaitMap_.clear();
  for (const auto& gaitName : gaitList_) {
    gaitMap_.insert(
        {gaitName, loadModeSequenceTemplate(gaitFile, gaitName, verbose)});
  }
  RCLCPP_INFO_STREAM(node->get_logger(),
                     robotName + "_mpc_mode_schedule command node is ready.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitKeyboardPublisher::publishKeyboardCommand(std::string& commandValue) {
  
  const auto commandLine = ocs2::stringToWords(commandValue);

  if (commandLine.empty()) {
    return;
  }

  if (commandLine.size() > 1) {
    std::cout << "WARNING: The command should be a single word." << std::endl;
    return;
  }

  // lower case transform
  auto gaitCommand = commandLine.front();
  std::transform(gaitCommand.begin(), gaitCommand.end(), gaitCommand.begin(),
                 ::tolower);

  if (gaitCommand == "list") {
    printGaitList(gaitList_);
    return;
  }

  try {
    ModeSequenceTemplate ModeSequenceTemplate = gaitMap_.at(gaitCommand);
    modeSequenceTemplatePublisher_->publish(
        createModeSequenceTemplateMsg(ModeSequenceTemplate));
    std::cout << "Gait publish succeed!!!.\n";
    std::cout << std::endl;
  } catch (const std::out_of_range& e) {
    std::cout << "Gait \"" << gaitCommand << "\" not found.\n";
    printGaitList(gaitList_);
  }
}

// /******************************************************************************************************/
// /******************************************************************************************************/
// /******************************************************************************************************/
// void GaitKeyboardPublisher::getKeyboardCommand() {
//   const std::string commadMsg =
//       "Enter the desired gait, for the list of available gait enter \"list\"";
//   std::cout << commadMsg << ": ";

//   auto shouldTerminate = []() { return !rclcpp::ok(); };
//   const auto commandLine = ocs2::stringToWords(ocs2::getCommandLineString(shouldTerminate));

//   if (commandLine.empty()) {
//     return;
//   }

//   if (commandLine.size() > 1) {
//     std::cout << "WARNING: The command should be a single word." << std::endl;
//     return;
//   }

//   // lower case transform
//   auto gaitCommand = commandLine.front();
//   std::transform(gaitCommand.begin(), gaitCommand.end(), gaitCommand.begin(),
//                  ::tolower);

//   if (gaitCommand == "list") {
//     printGaitList(gaitList_);
//     return;
//   }

//   try {
//     ModeSequenceTemplate ModeSequenceTemplate = gaitMap_.at(gaitCommand);
//     modeSequenceTemplatePublisher_->publish(
//         createModeSequenceTemplateMsg(ModeSequenceTemplate));
//   } catch (const std::out_of_range& e) {
//     std::cout << "Gait \"" << gaitCommand << "\" not found.\n";
//     printGaitList(gaitList_);
//   }
// }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitKeyboardPublisher::printGaitList(
    const std::vector<std::string>& gaitList) const {
  std::cout << "List of available gaits:\n";
  size_t itr = 0;
  for (const auto& s : gaitList) {
    std::cout << "[" << itr++ << "]: " << s << "\n";
  }
  std::cout << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/** Convert mode sequence template to ROS message */
// event_times defined in GaitModeSchedule.msg, switchingTimes defined in gait.info 
legged_msgs::msg::GaitModeSchedule GaitKeyboardPublisher::createModeSequenceTemplateMsg(
    const ModeSequenceTemplate& ModeSequenceTemplate) {
  legged_msgs::msg::GaitModeSchedule modeScheduleMsg;
  modeScheduleMsg.event_times.assign(
      ModeSequenceTemplate.switchingTimes.begin(),
      ModeSequenceTemplate.switchingTimes.end());
  modeScheduleMsg.mode_sequence.assign(
      ModeSequenceTemplate.modeSequence.begin(),
      ModeSequenceTemplate.modeSequence.end());
  return modeScheduleMsg;
}
