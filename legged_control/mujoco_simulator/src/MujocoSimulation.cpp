#include "mujoco_simulator/MujocoSimulation.hpp"

MujocoSimulation* MujocoSimulation::instance_ = nullptr;
bool MujocoSimulation::button_left = false;
bool MujocoSimulation::button_middle = false;
bool MujocoSimulation::button_right = false;
double MujocoSimulation::lastx = 0.0;
double MujocoSimulation::lasty = 0.0;

MujocoSimulation::MujocoSimulation(const rclcpp::Node::SharedPtr& node,
    const std::string& xmlFile,
    const std::string& simulatorFile)
    : node_(node), model_(nullptr), data_(nullptr), window_(nullptr)
{   
    instance_ = this;
    // Initialize MuJoCo visualization
    if (!glfwInit()) {
        mju_error("Could not initialize GLFW");
    }

    // Create window
    window_ = glfwCreateWindow(1280, 720, "MuJoCo Simulation", nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        mju_error("Could not create GLFW window");
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);  // Enable vsync

     // Initialize MuJoCo rendering
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context_);

    // load model
    loadModel(xmlFile, simulatorFile);

    // set redering;
    renderSetting(simulatorFile);

    // ros node
    state_pub_ = node_->create_publisher<legged_msgs::msg::SimulatorStateData>("simulator_state_data", 1);
    sensor_pub_ = node_->create_publisher<legged_msgs::msg::SimulatorSensorData>("simulator_sensor_data", 1);
    joint_control_sub_ = node_->create_subscription<legged_msgs::msg::JointControlData>(
        "joint_control_data", 1, std::bind(&MujocoSimulation::control_callback, this, std::placeholders::_1));
    start_control_server_ = node_->create_service<legged_msgs::srv::StartControl>("start_control", 
            std::bind(&MujocoSimulation::start_control_service, this, std::placeholders::_1, std::placeholders::_2));

}

MujocoSimulation::~MujocoSimulation() {
    // Free MuJoCo resources
    if (model_) mj_deleteModel(model_);
    if (data_) mj_deleteData(data_);

    // Free MuJoCo rendering context
    mjr_freeContext(&context_);

    // Terminate GLFW
    if (window_) glfwDestroyWindow(window_);
    glfwTerminate();
}

void MujocoSimulation::loadModel(const std::string& modelPath, const std::string& configFile) {
    // controller
    ocs2::loadData::loadCppDataType(configFile, "controller.timestep", timestep_);
    ocs2::loadData::loadCppDataType(configFile, "controller.wbc_control_frequency", control_frequency_);

    // pid
    ocs2::loadData::loadCppDataType(configFile, "pid.kp", Kp_);
    ocs2::loadData::loadCppDataType(configFile, "pid.kd", Kd_);

    // Load MuJoCo model and data
    model_ = mj_loadXML(modelPath.c_str(), nullptr, nullptr, 0);
    if (!model_) {
        mju_error("Failed to load model");
    }
    model_->opt.timestep = timestep_; //simulation timestep

    data_ = mj_makeData(model_);

    mjv_makeScene(model_, &scene_, 2000); 
    // Create MuJoCo context for rendering
    mjr_makeContext(model_, &context_, mjFONTSCALE_150);

    for (int i = 0; i < 12; ++i) {
        Joint_position_[i] = 0.0;
        Joint_velocity_[i] = 0.0;
        Joint_torque_[i] = 0.0;
    }
    
    Start_simulate_=true;
    Start_control_=false;

    std::vector<std::string> geom_names = {"floor", "LF_FOOT_geom", "RF_FOOT_geom", "LH_FOOT_geom", "RH_FOOT_geom"};
    for (const auto& name : geom_names) {
        int id = mj_name2id(model_, mjOBJ_GEOM, name.c_str());
        if (id != -1) {
            geom_ids_.push_back(id);
        }
    }
    for (int id : geom_ids_) {
        std::cout << "Geom ID: " << id << std::endl;
    }

}

void MujocoSimulation::renderSetting(const std::string& configFile)
{
    ocs2::loadData::loadCppDataType(configFile, "render.render_appearance", render_appearance_);
    ocs2::loadData::loadCppDataType(configFile, "render.render_inertia", render_inertia_);
    ocs2::loadData::loadCppDataType(configFile, "render.visualize_contacts", visualize_contacts_);
    ocs2::loadData::loadCppDataType(configFile, "render.tracking_base", tracking_base_);
    ocs2::loadData::loadCppDataType(configFile, "render.render_frequency", render_frequency_);

    if (tracking_base_) {
        const char* object_name = "trunk";
        int object_body_id = mj_name2id(model_, mjOBJ_BODY, object_name);
        cam_.type = mjCAMERA_TRACKING;
        cam_.trackbodyid = object_body_id;
    }
    if (visualize_contacts_) {
        opt_.flags[mjVIS_CONTACTPOINT] = 1; 
    }
    for (int i = 0; i < 3; ++i) {
        opt_.geomgroup[i] = 0;
    }
    opt_.geomgroup[0] = 1;
    if (render_appearance_) {
        opt_.geomgroup[1] = 1;
    }
    if (render_inertia_) {
        opt_.geomgroup[2] = 1;
    }

    glfwSetKeyCallback(window_, keyboard);
    glfwSetCursorPosCallback(window_, mouse_move);
    glfwSetMouseButtonCallback(window_, mouse_button);
    glfwSetScrollCallback(window_, scroll);
}

void MujocoSimulation::keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(instance_->model_, instance_->data_);
        mj_forward(instance_->model_, instance_->data_);
    }
}

void MujocoSimulation::mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    button_left   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    glfwGetCursorPos(window, &lastx, &lasty);
}

void MujocoSimulation::mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    if (!button_left && !button_middle && !button_right)
        return;

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    mjv_moveCamera(instance_->model_, action, dx / height, dy / height, &instance_->scene_, &instance_->cam_);
}

void MujocoSimulation::scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    mjv_moveCamera(instance_->model_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &instance_->scene_, &instance_->cam_);
}


void MujocoSimulation::run()
{
    rclcpp::Rate loop_rate(1000);
    simulateStep();
    render();

    mjtNum last_render_time = data_->time;
    int Render_count_=0;

    while (rclcpp::ok()) {
        // Render
        if (Render_count_>=50)
        {
            render();
            Render_count_=0;
        }
        Render_count_+=1;

        // Perform simulation step
        if (Start_control_) {
            // Reset *mujoco_sim.getSimuState()
            //mujoco_sim.render();

            Start_control_ = false;
            mjtNum simstart = data_->time;

            // Perform simulation for 1.0 / Control_frequency_ seconds
            while (data_->time - simstart < 1.0 / control_frequency_) {
                simulateStep();
                
            }
            // Render while robot is running
            if (data_->time - last_render_time >= 1.0 / render_frequency_) {
                render();
                last_render_time = data_->time;
            }
            
        }
        else if (Start_simulate_)
        {
            render();
            mjtNum simstart = data_->time;
            // Perform simulation for 1.0 / 100.0 seconds
            while (data_->time - simstart < 1.0 / control_frequency_) {
                simulateStep();
            }
            // Render while robot is running
            if (data_->time - last_render_time >= 1.0 / 100.0) {
                render();
                last_render_time = data_->time;
            }
        }
        rclcpp::spin_some(node_);
        loop_rate.sleep(); // Maintain 100Hz loop
    }

}

void MujocoSimulation::simulateStep() {
    if (!Start_simulate_)
    {
        double joint_position_value[12];
        double joint_velocity_value[12];
        double control_torque[12];
        //control sequence: FR,FL,RR,RL(hip, thigh, calf)
        for (int i = 0; i < 12; ++i) {
            joint_position_value[i] = data_->sensordata[i+10];
            joint_velocity_value[i] = data_->sensordata[i+22];
            control_torque[i] = static_cast<double>(Joint_torque_[i]) +
                Kp_ * (static_cast<double>(Joint_position_[i]) - joint_position_value[i]) +
                Kd_ * (static_cast<double>(Joint_velocity_[i]) - joint_velocity_value[i]);
            data_->ctrl[i] = control_torque[i];
            //std::cout << "data_->ctrl[" << i << "] = " << data_->ctrl[i] << std::endl;
        }
    }
    // Perform a simulation step
    mj_step(model_, data_);
}

void MujocoSimulation::render() {
    // Check if window exists
    if (window_) {
        // Get framebuffer size
        int width, height;
        glfwGetFramebufferSize(window_, &width, &height);

        if (width <= 0 || height <= 0) {
            RCLCPP_ERROR(node_->get_logger(), "Invalid framebuffer size: %d x %d", width, height);
            return;
        }

        mjrRect viewport = {0, 0, width, height};
        
        // Update the scene
        if (!model_ || !data_) {
            RCLCPP_ERROR(node_->get_logger(), "Model or data is null.");
            return;
        }

        mjv_updateScene(model_, data_, &opt_, nullptr, &cam_, mjCAT_ALL, &scene_);

        // Render the scene
        mjr_render(viewport, &scene_, &context_);

        // Swap buffers to display rendered image
        glfwSwapBuffers(window_);

        // Poll for and process events (e.g., keyboard input, window resizing)
        glfwPollEvents();
        //printf("Hello, World!\n");
    }
}


void MujocoSimulation::control_callback(legged_msgs::msg::JointControlData::SharedPtr msg)
{
    if (msg->joint_position.size() != 12) {
        RCLCPP_ERROR(node_->get_logger(), "Error: joint size is not 12. Current size: %zu", msg->joint_position.size());
        // rclcpp::shutdown();  // 结束程序
        // return;
    }
    Start_simulate_=false;
    Start_control_=true;

    // std::lock_guard<std::mutex> lock(bufferMutex_);
    for (size_t i = 0; i < msg->joint_position.size(); ++i) {
        Joint_position_[i] = msg->joint_position[i];
        Joint_velocity_[i] = msg->joint_velocity[i];
        Joint_torque_[i] = msg->joint_torque[i];
        //RCLCPP_INFO(node_->get_logger(), "  %f", Joint_torque_[i]);
    }
    publish_state_data();
    publish_sensor_data();
}


bool MujocoSimulation::checkCollision(const mjData* d, int geom1_id, int geom2_id) {
    for (int i = 0; i < d->ncon; ++i) {
        const mjContact& contact = d->contact[i];
        // std::cout << "Contact Info:\n";
        // std::cout << "  Geom1: " << contact.geom1 << "\n";
        // std::cout << "  Geom2: " << contact.geom2 << "\n";
        // std::cout << "  Distance: " << contact.dist << "\n";
        if ((contact.geom1 == geom1_id && contact.geom2 == geom2_id) ||
            (contact.geom1 == geom2_id && contact.geom2 == geom1_id)) {
            // std::cout << "Collision detected between geom " 
            //           << geom1_id << " and geom " << geom2_id 
            //           << ". Distance: " << contact.dist << std::endl;
            return 1;
        }
    }
    return 0;
}


void MujocoSimulation::publish_state_data()
{
    auto message = legged_msgs::msg::SimulatorStateData();
    message.simulation_time = data_->time;

    message.base_quat_values.clear();
    mjtNum* imu_data = &data_->sensordata[0];
    for (int i = 0; i < 4; ++i) {
        double quat_value;
        quat_value = static_cast<double>(data_->sensordata[i + 34]);
        message.base_quat_values.push_back(quat_value);
    }

    message.base_pose_values.clear();
    message.base_angvel_values.clear();
    message.base_linvel_values.clear();
    for (int i = 0; i < 3; ++i) {
        double base_pose_value;
        double base_angvel_value;
        double base_linvel_value;
        base_pose_value = static_cast<double>(data_->sensordata[i + 38]);
        message.base_pose_values.push_back(base_pose_value);
        base_angvel_value = static_cast<double>(data_->sensordata[i + 41]);
        message.base_angvel_values.push_back(base_angvel_value);
        base_linvel_value = static_cast<double>(data_->sensordata[i + 44]);
        message.base_linvel_values.push_back(base_linvel_value);
    }

    message.joint_position_values.clear();
    message.joint_velocity_values.clear();
    for (int i = 0; i < 12; ++i) {
        double joint_position_value;
        double joint_velocity_value;
        joint_position_value = static_cast<double>(data_->sensordata[i + 10]);
        message.joint_position_values.push_back(joint_position_value);
        joint_velocity_value = static_cast<double>(data_->sensordata[i + 22]);
        message.joint_velocity_values.push_back(joint_velocity_value);
    }

    message.contact_flags.resize(geom_ids_.size()-1);
    for (size_t i = 0; i < (geom_ids_.size()-1); ++i) {
        message.contact_flags[i] = checkCollision(data_, geom_ids_[0], geom_ids_[i+1]);
    }

    RCLCPP_INFO(node_->get_logger(),
            "Simulation time = [%f], \n"
            "Publishing base quat data: [%f, %f, %f, %f], \n"
            "Publishing base pos data: [%f, %f, %f], \n"
            "Publishing base angvel data: [%f, %f, %f], \n"
            "Publishing base linvel data: [%f, %f, %f], \n"
            "Publishing joint position data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
            "Publishing joint velocity data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
            "Publishing contact flag: [%zu, %zu, %zu, %zu]",
            message.simulation_time,
            message.base_quat_values[0], message.base_quat_values[1], 
            message.base_quat_values[2],message.base_quat_values[3],
            message.base_pose_values[0], message.base_pose_values[1],message.base_pose_values[2], 
            message.base_angvel_values[0], message.base_angvel_values[1],message.base_angvel_values[2], 
            message.base_linvel_values[0], message.base_linvel_values[1],message.base_linvel_values[2],
            message.joint_position_values[0], message.joint_position_values[1],
            message.joint_position_values[2], message.joint_position_values[3],
            message.joint_position_values[4], message.joint_position_values[5],
            message.joint_position_values[6], message.joint_position_values[7],
            message.joint_position_values[8], message.joint_position_values[9],
            message.joint_position_values[10], message.joint_position_values[11],
            message.joint_velocity_values[0], message.joint_velocity_values[1],
            message.joint_velocity_values[2], message.joint_velocity_values[3],
            message.joint_velocity_values[4], message.joint_velocity_values[5],
            message.joint_velocity_values[6], message.joint_velocity_values[7],
            message.joint_velocity_values[8], message.joint_velocity_values[9],
            message.joint_velocity_values[10], message.joint_velocity_values[11],
            static_cast<size_t>(message.contact_flags[0]), static_cast<size_t>(message.contact_flags[1]),
            static_cast<size_t>(message.contact_flags[2]), static_cast<size_t>(message.contact_flags[3]));
    RCLCPP_INFO(node_->get_logger(),
            "Simulation time = [%f]",message.simulation_time);

    state_pub_->publish(message);  
}


void MujocoSimulation::publish_sensor_data()
{
    auto message = legged_msgs::msg::SimulatorSensorData();
    message.simulation_time = data_->time;

    message.imu_quat_values.clear();
    message.imu_angvel_values.clear();
    message.imu_linacc_values.clear();
    for (int i = 0; i < 4; ++i) {
        double imu_quat_value;
        imu_quat_value=static_cast<double>(data_->sensordata[i]);
        message.imu_quat_values.push_back(imu_quat_value);
    }
    for (int i = 0; i < 3; ++i) {
        double imu_angvel_value;
        double imu_linacc_value;
        imu_angvel_value = static_cast<double>(data_->sensordata[i + 4]);
        imu_linacc_value = static_cast<double>(data_->sensordata[i + 7]);
        message.imu_angvel_values.push_back(imu_angvel_value);
        message.imu_linacc_values.push_back(imu_linacc_value);
    }

    message.joint_position_values.clear();
    message.joint_velocity_values.clear();
    for (int i = 0; i < 12; ++i) {
        double joint_position_value;
        double joint_velocity_value;
        joint_position_value = static_cast<double>(data_->sensordata[i + 10]);
        message.joint_position_values.push_back(joint_position_value);
        joint_velocity_value = static_cast<double>(data_->sensordata[i + 22]);
        message.joint_velocity_values.push_back(joint_velocity_value);
    }

    message.contact_flags.resize(geom_ids_.size()-1);
    for (size_t i = 0; i < (geom_ids_.size()-1); ++i) {
        message.contact_flags[i] = checkCollision(data_, geom_ids_[0], geom_ids_[i+1]);
    }

    // RCLCPP_INFO(node_->get_logger(),
    //         "Simulation time = [%f], \n"
    //         "Publishing IMU quat data: [%f, %f, %f, %f], \n"
    //         "Publishing IMU angvel data: [%f, %f, %f], \n"
    //         "Publishing IMU linacc data: [%f, %f, %f], \n"
    //         "Publishing joint position data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
    //         "Publishing joint velocity data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
    //         "Publishing contact flag: [%zu, %zu, %zu, %zu]",
    //         message.simulation_time,
    //         message.imu_quat_values[0], message.imu_quat_values[1], message.imu_quat_values[2],
    //         message.imu_quat_values[3],
    //         message.imu_angvel_values[0], message.imu_angvel_values[1], message.imu_angvel_values[2],
    //         message.imu_linacc_values[0], message.imu_linacc_values[1], message.imu_linacc_values[2],
    //         message.joint_position_values[0], message.joint_position_values[1],
    //         message.joint_position_values[2], message.joint_position_values[3],
    //         message.joint_position_values[4], message.joint_position_values[5],
    //         message.joint_position_values[6], message.joint_position_values[7],
    //         message.joint_position_values[8], message.joint_position_values[9],
    //         message.joint_position_values[10], message.joint_position_values[11],
    //         message.joint_velocity_values[0], message.joint_velocity_values[1],
    //         message.joint_velocity_values[2], message.joint_velocity_values[3],
    //         message.joint_velocity_values[4], message.joint_velocity_values[5],
    //         message.joint_velocity_values[6], message.joint_velocity_values[7],
    //         message.joint_velocity_values[8], message.joint_velocity_values[9],
    //         message.joint_velocity_values[10], message.joint_velocity_values[11],
    //         static_cast<size_t>(message.contact_flags[0]), static_cast<size_t>(message.contact_flags[1]),
    //         static_cast<size_t>(message.contact_flags[2]), static_cast<size_t>(message.contact_flags[3]));

    sensor_pub_->publish(message);  
}


void MujocoSimulation::start_control_service(const std::shared_ptr<legged_msgs::srv::StartControl::Request> request,
    std::shared_ptr<legged_msgs::srv::StartControl::Response> response)
{
    RCLCPP_INFO(node_->get_logger(), "Received: %s", request->start ? "true" : "false");

    //stop simulate and wait for control message
    Start_simulate_=false;

    legged_msgs::msg::SimulatorStateData state;

    state.simulation_time = data_->time;

    state.base_quat_values.clear();
    mjtNum* imu_data = &data_->sensordata[0];
    for (int i = 0; i < 4; ++i) {
        double quat_value;
        quat_value = static_cast<double>(data_->sensordata[i + 34]);
        state.base_quat_values.push_back(quat_value);
    }

    state.base_pose_values.clear();
    state.base_angvel_values.clear();
    state.base_linvel_values.clear();
    for (int i = 0; i < 3; ++i) {
        double base_pose_value;
        double base_angvel_value;
        double base_linvel_value;
        base_pose_value = static_cast<double>(data_->sensordata[i + 38]);
        state.base_pose_values.push_back(base_pose_value);
        base_angvel_value = static_cast<double>(data_->sensordata[i + 41]);
        state.base_angvel_values.push_back(base_angvel_value);
        base_linvel_value = static_cast<double>(data_->sensordata[i + 44]);
        state.base_linvel_values.push_back(base_linvel_value);
    }

    state.joint_position_values.clear();
    state.joint_velocity_values.clear();
    for (int i = 0; i < 12; ++i) {
        double joint_position_value;
        double joint_velocity_value;
        joint_position_value = static_cast<double>(data_->sensordata[i + 10]);
        state.joint_position_values.push_back(joint_position_value);
        joint_velocity_value = static_cast<double>(data_->sensordata[i + 22]);
        state.joint_velocity_values.push_back(joint_velocity_value);
    }

    state.contact_flags.resize(geom_ids_.size()-1);
    for (size_t i = 0; i < (geom_ids_.size()-1); ++i) {
        state.contact_flags[i] = checkCollision(data_, geom_ids_[0], geom_ids_[i+1]);
    }

    // response
    response->state = state;
    response->success = true;

    RCLCPP_INFO(node_->get_logger(),
            "Simulation time = [%f], \n"
            "Responding base quat data: [%f, %f, %f, %f], \n"
            "Responding base pos data: [%f, %f, %f], \n"
            "Responding base angvel data: [%f, %f, %f], \n"
            "Responding base linvel data: [%f, %f, %f], \n"
            "Responding joint position data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
            "Responding joint velocity data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
            "Responding contact flag: [%zu, %zu, %zu, %zu]",
            response->state.simulation_time,
            response->state.base_quat_values[0], response->state.base_quat_values[1], 
            response->state.base_quat_values[2],response->state.base_quat_values[3],
            response->state.base_pose_values[0], response->state.base_pose_values[1],response->state.base_pose_values[2], 
            response->state.base_angvel_values[0], response->state.base_angvel_values[1],response->state.base_angvel_values[2], 
            response->state.base_linvel_values[0], response->state.base_linvel_values[1],response->state.base_linvel_values[2],
            response->state.joint_position_values[0], response->state.joint_position_values[1],
            response->state.joint_position_values[2], response->state.joint_position_values[3],
            response->state.joint_position_values[4], response->state.joint_position_values[5],
            response->state.joint_position_values[6], response->state.joint_position_values[7],
            response->state.joint_position_values[8], response->state.joint_position_values[9],
            response->state.joint_position_values[10], response->state.joint_position_values[11],
            response->state.joint_velocity_values[0], response->state.joint_velocity_values[1],
            response->state.joint_velocity_values[2], response->state.joint_velocity_values[3],
            response->state.joint_velocity_values[4], response->state.joint_velocity_values[5],
            response->state.joint_velocity_values[6], response->state.joint_velocity_values[7],
            response->state.joint_velocity_values[8], response->state.joint_velocity_values[9],
            response->state.joint_velocity_values[10], response->state.joint_velocity_values[11],
            static_cast<size_t>(response->state.contact_flags[0]), static_cast<size_t>(response->state.contact_flags[1]),
            static_cast<size_t>(response->state.contact_flags[2]), static_cast<size_t>(response->state.contact_flags[3]));
}