#include "robot_handle.h"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>
#include <optional>
#include <regex>
#include <future>
#include <unordered_set>
#include <mutex>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include "database.h"
#include "i_dynamics_service.h"
#include "functional.hpp"

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <control_msgs/msg/dynamic_interface_group_values.hpp>
#include <controller_manager_msgs/srv/list_hardware_interfaces.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "driver_state_machine.h"
#include "io_module_store.h"
#include "motor_status_store.h"
#include "tool_frame_store.h"
#include "robot_model_store.h"


class RobotHandle::Impl {
public:
    rclcpp::Node::SharedPtr node_;
    std::optional<rclcpp::Logger> logger_;
    RobotModelStore model_store_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_publisher_;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr controller_state_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_torque_publisher_;
    rclcpp::Subscription<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr motor_driver_status_subscription_;
    rclcpp::Publisher<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr motor_driver_control_publisher_;
    rclcpp::Subscription<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr io_status_subscription_;
    rclcpp::Publisher<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr io_control_publisher_;
    rclcpp::TimerBase::SharedPtr driver_state_transition_timer_;

    IoModuleStore io_module_store_;
    MotorStatusStore motor_status_store_;
    uint64_t controller_update_period_; //ns
    ToolFrameStore tool_frame_store_;
    std::string grtp_path_;

    // expected end time for the last published trajectory
    rclcpp::Time expected_end_time_;
    bool has_expected_end_time_ = false;

    // running detection helpers
    size_t stationary_count_ = 0;
    size_t stationary_required_count_ = 0; // computed from settling time in constructor
    double stop_velocity_threshold_ = 1e-3; // rad/s
    double stop_position_threshold_ = 1e-3; // rad

    double cartesian_limits_max_trans_vel_{0.0};
    double cartesian_limits_max_trans_acc_{0.0};
    double cartesian_limits_max_trans_dec_{0.0};
    double cartesian_limits_max_rot_vel_{0.0};

    mutable std::mutex state_mutex_;

    rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult controller_result_;
    bool is_running_ = false;
    bool trajectory_complete_ = true;

    std::shared_ptr<IDynamicsService> dynamics_service_;

    const std::string CONTROL_WORD{"control_word"};
    const std::string STATUS_WORD{"status_word"};
    const std::string MODE{"mode"};

    // constructor helper
    void loadURDFFrames();
    void setupKinematicsJoints();
    void initDatabase();
    void loadCartesianLimits();
    void createROS2Source(); //create subs\client according to controllers' name
    void loadIOModulesName();
    void loadIOInterfacesNameFromService();
    std::optional<YAML::Node> loadGrtpYaml(bool log_as_error = false) const;

    KDL::Frame getFixedTransform(const KDL::Chain& chain);
    void sendDriverControlWord(const uint16_t& cw);
    void switchDriverMode(DriverMode mode);
    template<typename T>
    void sendDriverControlMessage(const std::string& name, const T& val);
    void enableMotorDrive();

    void moveJointByAbsPosition(const JointsPosition& joint_position, double velo_ratio);
    void executeTrajectory(trajectory_msgs::msg::JointTrajectory &msg);
    void setJointTorque(const JointsTorque& joint_torque);

    void jointStateCallback(const sensor_msgs::msg::JointState &msg);
    void controllerStateCallback(const control_msgs::msg::JointTrajectoryControllerState& msg);
    void motorDriverStatusCallback(const control_msgs::msg::DynamicInterfaceGroupValues& msg);
    void ioStatusCallback(const control_msgs::msg::DynamicInterfaceGroupValues& msg);

    void setIOState(const std::string& module_name, const std::string& interface_name, bool target_state);
};

std::optional<YAML::Node> RobotHandle::Impl::loadGrtpYaml(bool log_as_error) const {
    try {
        return YAML::LoadFile(grtp_path_);
    } catch (const std::exception& e) {
        if (log_as_error) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load YAML from %s: %s", grtp_path_.c_str(), e.what());
        } else {
            RCLCPP_WARN(node_->get_logger(), "Failed to load YAML from %s: %s", grtp_path_.c_str(), e.what());
        }
        return std::nullopt;
    }
}

void RobotHandle::Impl::loadURDFFrames() {
    auto urdf_string = AcquireParam<std::string>("/robot_state_publisher", "robot_description").value();
    if (!model_store_.model().initString(urdf_string)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to parse URDF model.");
        throw std::runtime_error("Failed to parse URDF");
    }
    RCLCPP_INFO(node_->get_logger(), "Successfully parsed URDF model.");

    std::vector<urdf::LinkSharedPtr> link_ptrs;
    model_store_.model().getLinks(link_ptrs);
    std::vector<std::string> available_names;
    for(const auto& l: link_ptrs) {
        available_names.push_back(l->name);
        RCLCPP_INFO(node_->get_logger(), "Got frame: %s", l->name.c_str());
    }
    // frame names will be resolved later by caller using RobotHandle helpers
}

void RobotHandle::Impl::setupKinematicsJoints() {
    auto root_opt = loadGrtpYaml();
    YAML::Node root = root_opt ? *root_opt : YAML::Node{};
    // If coordinate_system exists in YAML, read base and end effector frame names
    if (root && root["coordinate_system"] && root["coordinate_system"].IsMap()) {
        YAML::Node cs = root["coordinate_system"];
        model_store_.robotArmBaseLinkName() = cs["base_frame"].as<std::string>(model_store_.robotArmBaseLinkName());
        model_store_.robotArmEndLinkName() = cs["end_effector_frame"].as<std::string>(model_store_.robotArmEndLinkName());
    }

    // Build KDL structures from URDF
    kdl_parser::treeFromUrdfModel(model_store_.model(), model_store_.kdlTree());
    model_store_.kdlTree().getChain(model_store_.robotArmBaseLinkName(), model_store_.robotArmEndLinkName(), model_store_.kdlChain());
    for (const auto& seg : model_store_.kdlChain().segments) {
        auto joint = seg.getJoint();
        if(joint.getType() == KDL::Joint::JointType::Fixed) {continue;}
        Joint j{model_store_.model().getJoint(joint.getName()), 0};
        model_store_.jointNames().push_back(joint.getName());
        model_store_.currentJointPosition()[joint.getName()] = j;
        model_store_.currentJointVelocity()[joint.getName()] = j;
        model_store_.currentJointTorque()[joint.getName()] = j;
    }

    // Populate tool frames from YAML (if any) using the constructed KDL tree
    if (root && root["coordinate_system"] && root["coordinate_system"]["tool_frame"]) {
        YAML::Node tools = root["coordinate_system"]["tool_frame"];
        std::vector<std::string> tool_list;
        if (tools.IsSequence()) {
            for (const auto &t : tools) tool_list.push_back(t.as<std::string>());
        } else if (tools.IsScalar()) {
            tool_list.push_back(tools.as<std::string>());
        }
        if (!tool_list.empty()) {
            auto all_links = model_store_.kdlTree().getSegments();
            ToolInfo loaded_tools;
            KDL::Chain chain;
            for (const auto &v : tool_list) {
                if (!all_links.count(v)) {
                    RCLCPP_WARN(node_->get_logger(), "Tool frame %s does not exist in URDF; ignoring.", v.c_str());
                    continue;
                }
                model_store_.kdlTree().getChain(model_store_.robotArmEndLinkName(), v, chain);
                loaded_tools[v] = getFixedTransform(chain);
            }
            tool_frame_store_.initializeFromLoadedFrames(loaded_tools);
            if (tool_frame_store_.isToolFrameSet()) {
                RCLCPP_INFO(node_->get_logger(), "Loaded %zu tool frames from %s.", tool_frame_store_.getToolInfo().size(), grtp_path_.c_str());
            }
        }
    }
    else {
        RCLCPP_WARN(node_->get_logger(), "Tool frame not set");
    }
}

void RobotHandle::Impl::initDatabase() {
    auto root_opt = loadGrtpYaml();
    if (!root_opt) {
        return;
    }
    const YAML::Node& root = *root_opt;
    if (root["database_buffer_size"]) {
        int bs = root["database_buffer_size"].as<int>();
        DataBase::init(bs, model_store_.jointNames(), controller_update_period_);
        return;
    }
}

void RobotHandle::Impl::loadCartesianLimits() {
    auto root_opt = loadGrtpYaml();
    if (!root_opt) {
        return;
    }
    const YAML::Node& root = *root_opt;
    YAML::Node limits = root["cartesian_limits"];
    if (limits && limits.IsMap()) {
        cartesian_limits_max_trans_vel_ = limits["max_trans_vel"].as<double>(cartesian_limits_max_trans_vel_);
        cartesian_limits_max_trans_acc_ = limits["max_trans_acc"].as<double>(cartesian_limits_max_trans_acc_);
        cartesian_limits_max_trans_dec_ = limits["max_trans_dec"].as<double>(cartesian_limits_max_trans_dec_);
        cartesian_limits_max_rot_vel_ = limits["max_rot_vel"].as<double>(cartesian_limits_max_rot_vel_);
        RCLCPP_INFO(node_->get_logger(), "Loaded cartesian_limits from file %s.", grtp_path_.c_str());
    } else {
        RCLCPP_WARN(node_->get_logger(), "cartesian_limits not found in %s.", grtp_path_.c_str());
    }
}

void RobotHandle::Impl::createROS2Source() {
    auto root_opt = loadGrtpYaml(true);
    if (!root_opt) {
        throw std::runtime_error("failed to load grtp yaml");
    }
    const YAML::Node& root = *root_opt;
    YAML::Node controllers_name = root["controllers_name"];
    if (!controllers_name || !controllers_name.IsMap()) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid or missing controllers_name in %s.", grtp_path_.c_str());
        throw std::runtime_error("controllers_name must be a map in grtp yaml");
    }

    auto read_controller_name = [this, &controllers_name](const std::string& key, const std::string& fallback) {
        YAML::Node node = controllers_name[key];
        if (!node) {
            RCLCPP_WARN(this->node_->get_logger(), "controllers_name.%s missing, fallback to %s.", key.c_str(), fallback.c_str());
            return fallback;
        }
        if (node.IsScalar()) {
            return node.as<std::string>();
        }
        if (node.IsMap() && node["name"] && node["name"].IsScalar()) {
            return node["name"].as<std::string>();
        }

        RCLCPP_WARN(this->node_->get_logger(), "controllers_name.%s has unexpected type, fallback to %s.", key.c_str(), fallback.c_str());
        return fallback;
    };

    std::string position_controller_name = read_controller_name("position_controller", "position_controller");
    std::string torque_controller_name = read_controller_name("torque_controller", "torque_controller");
    std::string cia402_controller_name = read_controller_name("cia402_controller", "cia402_controller");
    std::string io_controller_name = read_controller_name("io_controller", "gpio_controller");


    joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&RobotHandle::Impl::jointStateCallback, this, std::placeholders::_1));
    controller_state_subscription_ = node_->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(position_controller_name + "/controller_state", 10, std::bind(&RobotHandle::Impl::controllerStateCallback, this, std::placeholders::_1));
    joint_traj_publisher_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(position_controller_name + "/joint_trajectory", 10);
    joint_torque_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(torque_controller_name + "/commands", 10);
    motor_driver_status_subscription_ = node_->create_subscription<control_msgs::msg::DynamicInterfaceGroupValues>(cia402_controller_name + "/gpio_states", 10, std::bind(&RobotHandle::Impl::motorDriverStatusCallback, this, std::placeholders::_1));
    motor_driver_control_publisher_ = node_->create_publisher<control_msgs::msg::DynamicInterfaceGroupValues>(cia402_controller_name + "/commands", 10);
    io_status_subscription_ = node_->create_subscription<control_msgs::msg::DynamicInterfaceGroupValues>(io_controller_name + "/gpio_states", 10, std::bind(&RobotHandle::Impl::ioStatusCallback, this, std::placeholders::_1));
    io_control_publisher_ = node_->create_publisher<control_msgs::msg::DynamicInterfaceGroupValues>(io_controller_name + "/commands", 10);
    
    driver_state_transition_timer_ = node_->create_wall_timer(std::chrono::nanoseconds(controller_update_period_), std::bind(&RobotHandle::Impl::enableMotorDrive, this));
    driver_state_transition_timer_->cancel();
}

void RobotHandle::Impl::loadIOModulesName()
{
    auto root_opt = loadGrtpYaml();
    if (!root_opt) {
        return;
    }
    const YAML::Node& root = *root_opt;
    YAML::Node io_group_name = root["io_group_name"];
    if (!io_group_name || !io_group_name["input"] || !io_group_name["output"]) {
        RCLCPP_WARN(node_->get_logger(), "io_group_name not found or incomplete in %s.", grtp_path_.c_str());
        return;
    }
    io_module_store_.initGroups(
        io_group_name["input"].as<std::vector<std::string>>(),
        io_group_name["output"].as<std::vector<std::string>>());
    loadIOInterfacesNameFromService();
}

void RobotHandle::Impl::loadIOInterfacesNameFromService()
{
    std::unordered_map<std::string, std::vector<std::string>> cmd_interfaces;
    std::unordered_map<std::string, std::unordered_set<std::string>> mon_interfaces;
    for (const auto &g : io_module_store_.inputGroupsName()) {
        cmd_interfaces[g] = {};
        mon_interfaces[g] = {};
    }
    for (const auto &g : io_module_store_.outputGroupsName()) {
        cmd_interfaces[g] = {};
        mon_interfaces[g] = {};
    }

    auto client = node_->create_client<controller_manager_msgs::srv::ListHardwareInterfaces>(
        "/controller_manager/list_hardware_interfaces");
    if (!client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_WARN(node_->get_logger(), "Service /controller_manager/list_hardware_interfaces is unavailable.");
        io_module_store_.setCommandInterfaces(cmd_interfaces);
        io_module_store_.setMonitorableInterfaces(mon_interfaces);
        return;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::ListHardwareInterfaces::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        RCLCPP_WARN(node_->get_logger(), "Calling /controller_manager/list_hardware_interfaces timed out or failed.");
        io_module_store_.setCommandInterfaces(cmd_interfaces);
        io_module_store_.setMonitorableInterfaces(mon_interfaces);
        return;
    }

    const auto response = future.get();
    auto parse_group_interface = [](const std::string &full_name, std::string &group, std::string &interface_name) {
        const auto separator = full_name.find('/');
        if (separator == std::string::npos || separator == 0 || separator + 1 >= full_name.size()) {
            return false;
        }
        group = full_name.substr(0, separator);
        interface_name = full_name.substr(separator + 1);
        return true;
    };

    for (const auto &hw_if : response->command_interfaces) {
        std::string group;
        std::string interface_name;
        if (!parse_group_interface(hw_if.name, group, interface_name)) {
            continue;
        }
        auto group_it = cmd_interfaces.find(group);
        if (group_it == cmd_interfaces.end()) {
            continue;
        }
        auto &ordered_interfaces = group_it->second;
        if (std::find(ordered_interfaces.begin(), ordered_interfaces.end(), interface_name) == ordered_interfaces.end()) {
            ordered_interfaces.push_back(interface_name);
        }
    }

    for (const auto &hw_if : response->state_interfaces) {
        std::string group;
        std::string interface_name;
        if (!parse_group_interface(hw_if.name, group, interface_name)) {
            continue;
        }
        auto group_it = mon_interfaces.find(group);
        if (group_it == mon_interfaces.end()) {
            continue;
        }
        group_it->second.insert(interface_name);
    }

    io_module_store_.setCommandInterfaces(cmd_interfaces);
    io_module_store_.setMonitorableInterfaces(mon_interfaces);
}

KDL::Frame RobotHandle::Impl::getFixedTransform(const KDL::Chain& chain)
{
    KDL::Frame T = KDL::Frame::Identity();
    for (const auto& seg: chain.segments) {
        if (seg.getJoint().getType() == KDL::Joint::None) {
            T = T * seg.pose(0.0);
        }
        else {
            throw(std::runtime_error("Tool should be fixed on end link!"));
        }
    }
    return T;
}

void RobotHandle::Impl::sendDriverControlWord(const uint16_t &cw)
{
    sendDriverControlMessage(CONTROL_WORD, cw);
}

void RobotHandle::Impl::switchDriverMode(DriverMode mode)
{
    const auto mode_value = static_cast<int8_t>(mode);
    switch(mode){
    case DriverMode::CSP: {
        JointsPosition current_position;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_position = model_store_.currentJointPosition();
        }
        moveJointByAbsPosition(current_position, 1.0);
        break;
    }
    case DriverMode::CST:{
        JointsPosition current_position;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_position = model_store_.currentJointPosition();
        }
        auto t_opt = dynamics_service_
                     ? dynamics_service_->currentPoseStableTorque(current_position)
                     : std::optional<JointsTorque>{};
        if(!t_opt.has_value()) {
            throw(std::runtime_error("Calculating rnea failed! Unable to switch to effort mode!"));
        }
        else {
            setJointTorque(t_opt.value());
        }
        break;
    }

    }
    sendDriverControlMessage(MODE, mode_value);
}

template<typename T>
void RobotHandle::Impl::sendDriverControlMessage(const std::string &name, const T &val)
{
    control_msgs::msg::DynamicInterfaceGroupValues msg;
    control_msgs::msg::InterfaceValue content;
    content.interface_names.push_back(name);
    content.values.push_back(val);
    for(const auto& joint_name : model_store_.jointNames()){
        msg.interface_groups.push_back(joint_name);
        msg.interface_values.push_back(content);
    }
    motor_driver_control_publisher_->publish(msg);
}

void RobotHandle::Impl::enableMotorDrive()
{
    const auto status_snapshot = motor_status_store_.status();
    if(std::all_of(status_snapshot.begin(), status_snapshot.end(),
                    [](const auto& s) { return s.second == DriverState::STATE_OPERATION_ENABLED; })) {
        driver_state_transition_timer_->cancel();
        return;
    }
    control_msgs::msg::DynamicInterfaceGroupValues msg;
    JointsTorque current_torque;
    JointsPosition current_position;
    std::vector<std::string> joint_names;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_torque = model_store_.currentJointTorque();
        current_position = model_store_.currentJointPosition();
        joint_names = model_store_.jointNames();
    }
    setJointTorque(current_torque);
    moveJointByAbsPosition(current_position, 1.0);
    for(const auto& joint_name: joint_names) {
        control_msgs::msg::InterfaceValue content;
        content.interface_names.push_back(CONTROL_WORD);
        auto it = status_snapshot.find(joint_name);
        if (it == status_snapshot.end()) continue;
        auto cw = DriverStateMachine::transitionControlWord(it->second, 0xf);
        if(cw == 0 ) {
            RCLCPP_WARN(node_->get_logger(), "Driver %s is in error! Please clear error first!", joint_name.c_str());
            driver_state_transition_timer_->cancel();
            return;
        }
        content.values.push_back(cw);
        msg.interface_groups.push_back(joint_name);
        msg.interface_values.push_back(content);
    }
    motor_driver_control_publisher_->publish(msg);
}

void RobotHandle::Impl::moveJointByAbsPosition(const JointsPosition& joint_position, double velo_ratio) {
    trajectory_msgs::msg::JointTrajectory msg;
    trajectory_msgs::msg::JointTrajectoryPoint p;
    JointsPosition current_position;
    std::vector<std::string> joint_names;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_position = model_store_.currentJointPosition();
        joint_names = model_store_.jointNames();
    }
    for(const auto& j_name : joint_names) {
        msg.joint_names.push_back(j_name);
        if(joint_position.find(j_name) != joint_position.end()) {
            p.positions.push_back(joint_position.at(j_name).joint_value);
        }
        else {
            p.positions.push_back(current_position.at(j_name).joint_value);
        }
        p.velocities.push_back(current_position.at(j_name).joint_info->limits->velocity * velo_ratio);
    }
    p.time_from_start = rclcpp::Duration::from_nanoseconds(
        static_cast<int64_t>(controller_update_period_));
    msg.points.push_back(p);
    // Streaming path: publish directly, no trajectory completion tracking
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        is_running_ = true;
        stationary_count_ = 0;
    }
    joint_traj_publisher_->publish(msg);
}

void RobotHandle::Impl::setJointTorque(const JointsTorque &joint_torque)
{
    std_msgs::msg::Float64MultiArray msg;
    std::vector<std::string> joint_names;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        joint_names = model_store_.jointNames();
    }
    for(const auto& joint_name : joint_names) {
        msg.data.push_back(joint_torque.at(joint_name).joint_value);
    }
    joint_torque_publisher_->publish(msg);
}

void RobotHandle::Impl::executeTrajectory(trajectory_msgs::msg::JointTrajectory &msg)
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        trajectory_complete_ = false;
        if (!msg.points.empty()) {
            rclcpp::Duration d(msg.points.back().time_from_start);
            expected_end_time_ = node_->get_clock()->now() + d;
            has_expected_end_time_ = true;
        }
        is_running_ = true;
        stationary_count_ = 0;
    }
    joint_traj_publisher_->publish(msg);
}

RobotHandle::RobotHandle(const std::shared_ptr<rclcpp::Node>& node)
    : impl_(std::make_unique<Impl>())
{
    impl_->node_ = node;
    impl_->logger_ = node->get_logger();

    auto p_path = AcquireParam<std::string>("/controller_manager", "grtp_path");
    if (p_path.has_value()) {
        impl_->grtp_path_ = p_path.value();
    }
    if (!impl_->grtp_path_.empty()) {
        RCLCPP_INFO(impl_->node_->get_logger(), "Resolved grtp_path: %s", impl_->grtp_path_.c_str());
    } else {
        RCLCPP_WARN(impl_->node_->get_logger(), "grtp_path not set; file-based loads will be skipped.");
    }

    auto update_rate = AcquireParam<int32_t>("/controller_manager", "update_rate").value();
    impl_->controller_update_period_ = 1e9 / update_rate;

    // 100ms settling window for trajectory completion detection
    impl_->stationary_required_count_ = static_cast<size_t>(
        std::ceil(100000000.0 / impl_->controller_update_period_));

    impl_->loadURDFFrames();
    impl_->setupKinematicsJoints();
    impl_->initDatabase();
    impl_->loadCartesianLimits();
    impl_->createROS2Source();
    impl_->loadIOModulesName();

    impl_->sendDriverControlMessage(impl_->MODE, static_cast<int8_t>(DriverMode::CSP));

}

RobotHandle::~RobotHandle() = default;


const urdf::Model& RobotHandle::getURDFModel() const {
    return impl_->model_store_.model();
}

const KDL::Chain& RobotHandle::getKDLChain() const {
    return impl_->model_store_.kdlChain();
}

JointsPosition RobotHandle::getCurrentJointPosition() const {
    std::lock_guard<std::mutex> lock(impl_->state_mutex_);
    return impl_->model_store_.currentJointPosition();
}

JointsVelocity RobotHandle::getCurrentJointVelocity() const {
    std::lock_guard<std::mutex> lock(impl_->state_mutex_);
    return impl_->model_store_.currentJointVelocity();
}

JointsTorque RobotHandle::getCurrentJointTorque() const
{
    std::lock_guard<std::mutex> lock(impl_->state_mutex_);
    return impl_->model_store_.currentJointTorque();
}

// helper function

size_t RobotHandle::getJointNums() const {
    return impl_->model_store_.jointNames().size();
}

double RobotHandle::getJointVelocityLimit(const std::string& j_name) const {
    std::lock_guard<std::mutex> lock(impl_->state_mutex_);
    return impl_->model_store_.currentJointPosition().at(j_name).joint_info->limits->velocity;
}

double RobotHandle::getJointLowerLimit(const std::string& j_name) const {
    std::lock_guard<std::mutex> lock(impl_->state_mutex_);
    return impl_->model_store_.currentJointPosition().at(j_name).joint_info->limits->lower;
}

double RobotHandle::getJointUpperLimit(const std::string& j_name) const {
    std::lock_guard<std::mutex> lock(impl_->state_mutex_);
    return impl_->model_store_.currentJointPosition().at(j_name).joint_info->limits->upper;
}

// Strictly follow kinematic chain' order
const std::vector<std::string>& RobotHandle::getJointsName() const {
    return impl_->model_store_.jointNames();
}

auto RobotHandle::getTime() -> rclcpp::Time {
    return impl_->node_->get_clock()->now();
}

auto RobotHandle::getCartesianLimitsMaxTransVel() const -> double {
    return impl_->cartesian_limits_max_trans_vel_;
}

auto RobotHandle::getCartesianLimitsMaxTransAcc() const -> double {
    return impl_->cartesian_limits_max_trans_acc_;
}

auto RobotHandle::getCartesianLimitsMaxTransDec() const -> double {
    return impl_->cartesian_limits_max_trans_dec_;
}

auto RobotHandle::getCartesianLimitsMaxRotVel() const -> double {
    return impl_->cartesian_limits_max_rot_vel_;
}

uint64_t RobotHandle::getControllerUpdatePeriod() const {
    return impl_->controller_update_period_;
}

const std::string& RobotHandle::getRobotArmBaseLinkName() const {
    return impl_->model_store_.robotArmBaseLinkName();
}

const std::string& RobotHandle::getRobotArmEndLinkName() const {
    return impl_->model_store_.robotArmEndLinkName();
}

const ToolInfo& RobotHandle::getRobotArmToolInfo() const {
    return impl_->tool_frame_store_.getToolInfo();
}

void RobotHandle::moveJointByVelocity(const JointsVelocity&) {
    RCLCPP_ERROR(impl_->node_->get_logger(), "Deprecated interface!");
}

void RobotHandle::moveJointByAbsPosition(const JointsPosition &joint_position, double velo_ratio)
{
    impl_->moveJointByAbsPosition(joint_position, velo_ratio);
}

void RobotHandle::executeTrajectory(trajectory_msgs::msg::JointTrajectory &trajectory)
{
    impl_->executeTrajectory(trajectory);
}

void RobotHandle::setJointTorque(const JointsTorque &joint_torque)
{
    impl_->setJointTorque(joint_torque);
}

void RobotHandle::deleteToolFrame(const std::string& tool_name) {
    impl_->tool_frame_store_.deleteToolFrame(tool_name);
}

void RobotHandle::addToolFrame(const std::string& tool_name, const KDL::Frame& frame) {
    impl_->tool_frame_store_.addToolFrame(tool_name, frame);
}

void RobotHandle::setCurrentToolFrame(const std::string& tool_name) {
    impl_->tool_frame_store_.setCurrentToolFrame(tool_name);
}

const bool& RobotHandle::isToolFrameSet() const {
    return impl_->tool_frame_store_.isToolFrameSet();
}

const std::string& RobotHandle::getCurrentToolFrame() const {
    return impl_->tool_frame_store_.getCurrentToolFrame();
}

void RobotHandle::Impl::jointStateCallback(const sensor_msgs::msg::JointState &msg)
{
    auto& data_base = DataBase::instance();
    for(size_t i = 0; i < msg.name.size(); ++i) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            model_store_.currentJointPosition()[msg.name[i]].joint_value = msg.position[i];
            model_store_.currentJointVelocity()[msg.name[i]].joint_value = msg.velocity[i];
            model_store_.currentJointTorque()[msg.name[i]].joint_value = msg.effort[i];
        }
        data_base.appendData(DataTypeEnum::POSITION, msg.name[i], msg.position[i]);
        data_base.appendData(DataTypeEnum::VELOCITY, msg.name[i], msg.velocity[i]);
        data_base.appendData(DataTypeEnum::TORQUE, msg.name[i], msg.effort[i]);
    }
}

void RobotHandle::Impl::controllerStateCallback(const control_msgs::msg::JointTrajectoryControllerState &msg)
{
    // Determine whether the controller/robot is still executing a trajectory.
    // Strategy: if velocities and position error are below thresholds for
    // `stationary_required_count_` consecutive callbacks, consider stopped.
    double max_vel = 0.0;
    double max_pos_err = 0.0;

    // use JTC message fields: `reference` is the setpoint, `feedback` is current
    const auto &feedback = msg.feedback;
    const auto &reference = msg.reference;

    size_t n = std::min(feedback.positions.size(), feedback.velocities.size());
    // if velocities available, use them; otherwise fallback to position deltas
    if (n > 0 && reference.positions.size() >= n) {
        for (size_t i = 0; i < n; ++i) {
            double v = std::abs(feedback.velocities[i]);
            max_vel = std::max(max_vel, v);
            double p_err = std::abs(reference.positions[i] - feedback.positions[i]);
            max_pos_err = std::max(max_pos_err, p_err);
        }
    } else if (feedback.positions.size() > 0 && reference.positions.size() == feedback.positions.size()) {
        for (size_t i = 0; i < feedback.positions.size(); ++i) {
            double p_err = std::abs(reference.positions[i] - feedback.positions[i]);
            max_pos_err = std::max(max_pos_err, p_err);
        }
        // without velocity info, be conservative about stopping
        max_vel = stop_velocity_threshold_ * 2.0;
    } else {
        // no usable data; keep previous state
        return;
    }

    if (max_vel > stop_velocity_threshold_ || max_pos_err > stop_position_threshold_) {
        // still moving / not at target
        std::lock_guard<std::mutex> lock(state_mutex_);
        stationary_count_ = 0;
        is_running_ = true;
    } else {
        std::lock_guard<std::mutex> lock(state_mutex_);
        stationary_count_++;
        if (stationary_count_ >= stationary_required_count_) {
            // only clear running if we are past the expected end time (with slack)
            if (has_expected_end_time_) {
                auto now = node_->get_clock()->now();
                if (now >= expected_end_time_) {
                    is_running_ = false;
                    trajectory_complete_ = true;
                    has_expected_end_time_ = false;
                } else {
                    // still within expected execution window; keep running
                    is_running_ = true;
                }
            } else {
                is_running_ = false;
            }
        }
    }
}

void RobotHandle::Impl::motorDriverStatusCallback(const control_msgs::msg::DynamicInterfaceGroupValues &msg)
{
    motor_status_store_.processStatusMessage(msg, model_store_, node_->get_logger());
}

void RobotHandle::Impl::ioStatusCallback(const control_msgs::msg::DynamicInterfaceGroupValues &msg)
{
    io_module_store_.processStatusMessage(msg, node_->get_logger());
}

void RobotHandle::Impl::setIOState(const std::string &module_name, const std::string &interface_name, bool target_state)
{
    if (!io_control_publisher_) {
        RCLCPP_WARN(node_->get_logger(), "IO control publisher is not available.");
        return;
    }

    control_msgs::msg::DynamicInterfaceGroupValues msg;
    control_msgs::msg::InterfaceValue interface_value;
    interface_value.interface_names.push_back(interface_name);
    interface_value.values.push_back(target_state ? 1.0 : 0.0);
    msg.interface_groups.push_back(module_name);
    msg.interface_values.push_back(interface_value);
    io_control_publisher_->publish(msg);
}

//warpper function
void RobotHandle::setIsRunning(bool is_running)
{
    std::lock_guard<std::mutex> lock(impl_->state_mutex_);
    impl_->is_running_ = is_running;
}

bool RobotHandle::isRunning() const
{
    std::lock_guard<std::mutex> lock(impl_->state_mutex_);
    return impl_->is_running_;
}

bool RobotHandle::isTrajectoryComplete() const
{
    std::lock_guard<std::mutex> lock(impl_->state_mutex_);
    return impl_->trajectory_complete_;
}

void RobotHandle::setDynamicsService(const std::shared_ptr<IDynamicsService>& dynamics)
{
    impl_->dynamics_service_ = dynamics;
}

size_t RobotHandle::registerMotorStatusCallback(MotorStatusCallback cb)
{
    return impl_->motor_status_store_.registerStatusCallback(std::move(cb));
}

void RobotHandle::unregisterMotorStatusCallback(size_t callback_id)
{
    impl_->motor_status_store_.unregisterStatusCallback(callback_id);
}

void RobotHandle::disableMotorDrive()
{
    impl_->sendDriverControlMessage(impl_->CONTROL_WORD, 0x07);
}

void RobotHandle::clearFault()
{
    impl_->sendDriverControlMessage(impl_->CONTROL_WORD, 0x80);
}

void RobotHandle::enableMotorDrive()
{
    impl_->driver_state_transition_timer_->reset();
}

void RobotHandle::switchToCSP()
{
    impl_->switchDriverMode(DriverMode::CSP);
}

void RobotHandle::switchToCST()
{
    impl_->switchDriverMode(DriverMode::CST);
}

const std::vector<std::string>& RobotHandle::getIOInputGroupsName() const
{
    return impl_->io_module_store_.inputGroupsName();
}

const std::vector<std::string>& RobotHandle::getIOOutputGroupsName() const
{
    return impl_->io_module_store_.outputGroupsName();
}

const std::vector<std::string>& RobotHandle::getIOInterfacesName(const std::string &module_name) const
{
    return impl_->io_module_store_.interfacesName(module_name);
}

bool RobotHandle::isIOMonitorable(const std::string &module_name, const std::string &interface_name) const
{
    return impl_->io_module_store_.isMonitorable(module_name, interface_name);
}

size_t RobotHandle::registerIOStatusCallback(IOStatusCallback cb)
{
    return impl_->io_module_store_.registerStatusCallback(std::move(cb));
}

void RobotHandle::unregisterIOStatusCallback(size_t callback_id)
{
    impl_->io_module_store_.unregisterStatusCallback(callback_id);
}

void RobotHandle::setIOState(const std::string &module_name, const std::string &interface_name, bool target_state)
{
    impl_->setIOState(module_name, interface_name, target_state);
}
