#ifndef MUJOCO_SIMULATION_HPP
#define MUJOCO_SIMULATION_HPP

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <GLFW/glfw3.h>  // For rendering and window handling
#include <mutex>
#include <ocs2_core/misc/LoadData.h>
//#include "std_msgs/msg/float32.hpp"
#include "legged_msgs/msg/simulator_state_data.hpp"
#include "legged_msgs/msg/simulator_sensor_data.hpp"
#include "legged_msgs/msg/joint_control_data.hpp"
#include "legged_msgs/srv/start_control.hpp"

class MujocoSimulation
{
public:
    MujocoSimulation(const rclcpp::Node::SharedPtr& node, 
    const std::string& xmlFile,
    const std::string& simulatorFile);
    ~MujocoSimulation();
    
    mjData* getData() { return data_; }
    bool* getSimuState() { return &Start_simulate_; }
    bool* getContrlState() { return &Start_control_; }
    void loadModel(const std::string& modelPath, const std::string& configPath);
    //void copyControlFromBuffer();
    void simulateStep();
    void renderSetting(const std::string& configFile);
    void render();
    void control_callback(const legged_msgs::msg::JointControlData::SharedPtr msg);
    bool checkCollision(const mjData* d, int geom1_id, int geom2_id);
    void run();
    void publish_state_data();
    void publish_sensor_data();
    void start_control_service(const std::shared_ptr<legged_msgs::srv::StartControl::Request> request,
        std::shared_ptr<legged_msgs::srv::StartControl::Response> response);

    static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
    static void mouse_button(GLFWwindow* window, int button, int act, int mods);
    static void mouse_move(GLFWwindow* window, double xpos, double ypos);
    static void scroll(GLFWwindow* window, double xoffset, double yoffset);

    

private:
    static MujocoSimulation* instance_; // 静态实例指针
    
    rclcpp::Node::SharedPtr node_;
    mjModel* model_;
    mjData* data_;
    mjvScene scene_;
    mjvCamera cam_;
    mjvOption opt_;
    mjrContext context_;

    // rendering
    bool render_appearance_;
    bool render_inertia_;
    bool visualize_contacts_;
    bool tracking_base_;
    double render_frequency_;

    // GLFW window
    GLFWwindow* window_;

    // ros2
    rclcpp::Publisher<legged_msgs::msg::SimulatorStateData>::SharedPtr state_pub_; //real state
    rclcpp::Publisher<legged_msgs::msg::SimulatorSensorData>::SharedPtr sensor_pub_; // sensor date
    rclcpp::Subscription<legged_msgs::msg::JointControlData>::SharedPtr joint_control_sub_;
    rclcpp::Service<legged_msgs::srv::StartControl>::SharedPtr start_control_server_;

    // motor control
    double Joint_position_buffer_[12];
    double Joint_velocity_buffer_[12];
    double Joint_torque_buffer_[12];
    double Joint_position_[12];
    double Joint_velocity_[12];
    double Joint_torque_[12];
    double timestep_;
    double control_frequency_;
    double Kp_;
    double Kd_;
    bool Start_control_=false;
    bool Start_simulate_=false;

    // contact flag
    std::vector<int> geom_ids_; // floor, LF_FOOT, RF_FOOT, LH_FOOT, RH_FOOT
    //int geom_id_[5]; // floor, LF_FOOT, RF_FOOT, LH_FOOT, RH_FOOT

    static bool button_left, button_middle, button_right;
    static double lastx, lasty;
};

#endif  // MUJOCO_SIMULATION_H
