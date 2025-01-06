#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/Display.h>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include "user_command/goal/TargetTrajectoriesKeyboardPublisher.h"
#include "user_command/gait/GaitKeyboardPublisher.h"
#include "rclcpp/rclcpp.hpp"


// Split command line
std::vector<std::string> splitByColon(const std::string& input) {
    std::vector<std::string> result;
    std::string token;
    std::istringstream tokenStream(input);


    while (std::getline(tokenStream, token, ':')) {
        result.push_back(token);
    }
    return result;
}

// erase the space in the front of command line
std::string trimLeft(const std::string& str) {
    std::string result = str;
    result.erase(result.begin(), std::find_if(result.begin(), result.end(), [](unsigned char ch) {
        return !std::isspace(ch);  
    }));
    return result;
}


int main(int argc, char* argv[]) {
  const std::string robotName = "legged_robot";

  // Initialize ros node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
      robotName + "_user_command",
      rclcpp::NodeOptions()
          .allow_undeclared_parameters(true)
          .automatically_declare_parameters_from_overrides(true));

  // // Get node parameters-from launch
  const std::string gaitCommandFile =
      node->get_parameter("gaitCommandFile").as_string();
  std::cerr << "Loading gait file from launch: " << gaitCommandFile << std::endl;

  // const std::string gaitCommandFile = "/home/zhx/Desktop/zhx_legged_ocs2_ros2/src/legged_control/user_command/config/a1/gait.info";
  // std::cerr << "Loading gait file: " << gaitCommandFile << std::endl;
  GaitKeyboardPublisher gaitCommand(node, gaitCommandFile, robotName, true);

  const ocs2::scalar_array_t relativeBaseLimit{10.0, 10.0, 0.2, 360.0};
  // const std::string referenceFile = "/home/zhx/Desktop/zhx_legged_ocs2_ros2/src/legged_control/user_command/config/a1/reference.info";
  const std::string referenceFile =
      node->get_parameter("referenceFile").as_string();
    std::cerr << "Loading reference file from launch: " << referenceFile << std::endl;
  TargetTrajectoriesKeyboardPublisher targetPoseCommand(node, robotName, relativeBaseLimit, referenceFile);

  const std::string commadMsg =
    "Enter 'gait:xxx' for the desired gait,\n"
    "Enter 'gait:list' for the list of available gaits,\n"
    "Enter 'goal:x x x x' for the desired goal,Enter XYZ and Yaw (deg) displacements, separated by spaces.\n";
    
  while (rclcpp::ok()) {
    std::cout << commadMsg << ": ";

    auto shouldTerminate = []() { return !rclcpp::ok(); };
    const auto commandLine = ocs2::getCommandLineString(shouldTerminate);

    std::vector<std::string> splitInput = splitByColon(commandLine);
    //gaitCommand.getKeyboardCommand();
    if (splitInput.size() == 2) {
      std::string commandType = splitInput[0];  
      std::string commandValue = trimLeft(splitInput[1]);
      //std::string commandValue = splitInput[1]; 
      RCLCPP_INFO(node->get_logger(), "Command Type: %s", commandType.c_str());
      RCLCPP_INFO(node->get_logger(), "Command Value: %s", commandValue.c_str());
      // std::cout << "Command Type: " << commandType << std::endl;
      // std::cout << "Command Value: " << commandValue << std::endl;
      if (commandType == "gait")
      {
        //std::cerr << "gait!!! " << std::endl;
        gaitCommand.publishKeyboardCommand(commandValue);
      }
      else if (commandType == "goal")
      {
        //std::cerr << "goal!!! " << std::endl;
        rclcpp::spin_some(node);
        targetPoseCommand.publishKeyboardCommand(commandValue);
        
      }
      else{
        RCLCPP_WARN(node->get_logger(), "Invalid command. Please use correct command.");
        std::cout << std::endl;
      }
    }
    else 
    {
      RCLCPP_WARN(node->get_logger(), "Invalid input format. Please use the format 'command:value'.");
      std::cout << std::endl;
    }
  }
  // Successful exit
  //std::cerr << "Succeed " << std::endl;
  return 0;
}
