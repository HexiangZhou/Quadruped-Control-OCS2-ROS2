#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mujoco_simulator/MujocoSimulation.hpp"


int main(int argc, char** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    const std::string robotName = "legged_robot";
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
        robotName + "_simulator",
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true));

    const std::string xmlFile = node->get_parameter("xmlFile").as_string();
    const std::string simulatorFile = node->get_parameter("simulatorFile").as_string();
    // const std::string xmlFile = "/home/zhx/Desktop/zhx_legged_ocs2_master/src/legged_control/mujoco_simulator/models/b1/urdf/robot.xml";
    // const std::string simulatorFile = "/home/zhx/Desktop/zhx_legged_ocs2_master/src/legged_control/user_command/config/b1/simulation.info";

    // Create MujocoSimulation instance
    MujocoSimulation mujoco_sim(node, xmlFile, simulatorFile);

    mujoco_sim.run();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
