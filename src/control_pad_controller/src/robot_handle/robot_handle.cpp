#include "robot_handle.h"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>
#include <optional>
#include <regex>
#include <future>
#include <unordered_set>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include "database.h"
#include "dynamic_plugin.h"
#include "singleton.hpp"
#include "functional.hpp"

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <control_msgs/msg/dynamic_interface_group_values.hpp>
#include <controller_manager_msgs/srv/list_hardware_interfaces.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>


class RobotHandle::Impl {
public:
    rclcpp::Node::SharedPtr node_;
    std::optional<rclcpp::Logger> logger_;
    std::shared_ptr<RobotHandle> singleton_;
    urdf::Model model_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_publisher_;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr controller_state_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_torque_publisher_;
    rclcpp::Subscription<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr motor_driver_status_subscription_;
    rclcpp::Publisher<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr motor_driver_control_publisher_;
    rclcpp::Subscription<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr io_status_subscription_;
    rclcpp::Publisher<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr io_control_publisher_;
    rclcpp::TimerBase::SharedPtr driver_state_transition_timer_;

    std::vector<std::string> io_input_groups_name_;
    std::vector<std::string> io_output_groups_name_;
    std::unordered_map<std::string, std::vector<std::string>> io_group_command_interfaces_;
    std::unordered_map<std::string, std::unordered_set<std::string>> io_group_monitorable_state_interfaces_;

    uint64_t controller_update_period_; //ns
    std::vector<std::string> joint_names_;
    JointsPosition current_joint_position_;
    JointsVelocity current_joint_velocity_;
    JointsTorque current_joint_torque_;
    JointsTorque current_joint_estimated_external_torque_;
    JointsTorque joint_torque_offset_; //just use for observer test
    std::string robot_arm_base_link_name_;
    std::string robot_arm_end_link_name_;
    ToolInfo robot_arm_tool_info_;
    bool tool_frame_set_;
    std::string current_tool_frame_;
    std::string grtp_path_;

    // expected end time for the last published trajectory
    rclcpp::Time expected_end_time_;
    bool has_expected_end_time_ = false;

    // running detection helpers
    size_t stationary_count_ = 0;
    size_t stationary_required_count_ = 0; // require N consecutive stationary updates
    double stop_velocity_threshold_ = 1e-3; // rad/s
    double stop_position_threshold_ = 1e-3; // rad

    double cartisian_limits_max_trans_vel_;
    double cartisian_limits_max_trans_acc_;
    double cartisian_limits_max_trans_dec_;
    double cartisian_limits_max_rot_vel_;
    DragParams drag_params_;

    JointsMode current_joint_mode_;
    JointsStatus current_joint_status_;
    std::unordered_map<size_t, MotorStatusCallback> motor_status_callbacks_;
    std::mutex motor_status_callbacks_mutex_;
    size_t motor_status_callback_next_id_ = 1;
    IOStatus io_status_;
    std::unordered_map<size_t, IOStatusCallback> io_status_callbacks_;
    std::mutex io_status_callbacks_mutex_;
    size_t io_status_callback_next_id_ = 1;

    rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult controller_result_;
    bool is_running_ = false;

    const std::string CONTROL_WORD{"control_word"};
    const std::string STATUS_WORD{"status_word"};
    const std::string MODE{"mode"};

    // constructor helper
    void loadURDFFrames();
    void setupKinematicsJoints();
    void initDatabase();
    void loadCartesianLimits();
    void loadDragParams();
    void createROS2Source(); //create subs\client according to controllers' name
    void loadIOModulesName();
    void loadIOInterfacesNameFromService();

    KDL::Frame getFixedTransform(const KDL::Chain& chain);
    DriverState getDriverState(uint16_t status_word);
    uint16_t ciA402Transition(DriverState state, uint16_t control_word);
    void sendDriverControlWord(const uint16_t& cw);
    void switchDriverMode(const int8_t& mode);
    template<typename T>
    void sendDriverControlMessage(const std::string& name, const T& val);
    void enableMotorDrive();

    void moveJointByAbsPosition(trajectory_msgs::msg::JointTrajectory &msg);
    void moveJointByAbsPosition(const JointsPosition& joint_position, double velo_ratio);
    void setJointTorque(const JointsTorque& joint_torque);

    void jointStateCallback(const sensor_msgs::msg::JointState &msg);
    void controllerStateCallback(const control_msgs::msg::JointTrajectoryControllerState& msg);
    void motorDriverStatusCallback(const control_msgs::msg::DynamicInterfaceGroupValues& msg);
    void ioStatusCallback(const control_msgs::msg::DynamicInterfaceGroupValues& msg);

    size_t registerMotorStatusCallback(MotorStatusCallback cb);
    void unregisterMotorStatusCallback(size_t callback_id);
    size_t registerIOStatusCallback(IOStatusCallback cb);
    void unregisterIOStatusCallback(size_t callback_id);
    void setIOState(const std::string& module_name, const std::string& interface_name, bool target_state);
};

void RobotHandle::Impl::loadURDFFrames() {
    auto urdf_string = AcquireParam<std::string>("/robot_state_publisher", "robot_description").value();
    if (!model_.initString(urdf_string)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to parse URDF model.");
        throw std::runtime_error("Failed to parse URDF");
    }
    RCLCPP_INFO(node_->get_logger(), "Successfully parsed URDF model.");

    std::vector<urdf::LinkSharedPtr> link_ptrs;
    model_.getLinks(link_ptrs);
    std::vector<std::string> available_names;
    for(const auto& l: link_ptrs) {
        available_names.push_back(l->name);
        RCLCPP_INFO(node_->get_logger(), "Got frame: %s", l->name.c_str());
    }
    // frame names will be resolved later by caller using RobotHandle helpers
}

void RobotHandle::Impl::setupKinematicsJoints() {
    YAML::Node root = YAML::LoadFile(grtp_path_);;
    // If coordinate_system exists in YAML, read base and end effector frame names
    if (root && root["coordinate_system"] && root["coordinate_system"].IsMap()) {
        YAML::Node cs = root["coordinate_system"];
        robot_arm_base_link_name_ = cs["base_frame"].as<std::string>(robot_arm_base_link_name_);
        robot_arm_end_link_name_ = cs["end_effector_frame"].as<std::string>(robot_arm_end_link_name_);
    }

    // Build KDL structures from URDF
    kdl_parser::treeFromUrdfModel(model_, kdl_tree_);
    kdl_tree_.getChain(robot_arm_base_link_name_, robot_arm_end_link_name_, kdl_chain_);
    for (const auto& seg : kdl_chain_.segments) {
        auto joint = seg.getJoint();
        if(joint.getType() == KDL::Joint::JointType::Fixed) {continue;}
        Joint j{model_.getJoint(joint.getName()), 0};
        joint_names_.push_back(joint.getName());
        current_joint_position_[joint.getName()] = j;
        current_joint_velocity_[joint.getName()] = j;
        current_joint_torque_[joint.getName()] = j;
        current_joint_estimated_external_torque_[joint.getName()] = j;
        joint_torque_offset_[joint.getName()] = j;
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
            auto all_links = kdl_tree_.getSegments();
            KDL::Chain chain;
            for (const auto &v : tool_list) {
                if (!all_links.count(v)) {
                    RCLCPP_WARN(node_->get_logger(), "Tool frame %s does not exist in URDF; ignoring.", v.c_str());
                    continue;
                }
                kdl_tree_.getChain(robot_arm_end_link_name_, v, chain);
                robot_arm_tool_info_[v] = getFixedTransform(chain);
            }
            if (!robot_arm_tool_info_.empty()) {
                tool_frame_set_ = true;
                current_tool_frame_ = robot_arm_tool_info_.begin()->first;
                RCLCPP_INFO(node_->get_logger(), "Loaded %zu tool frames from %s.", robot_arm_tool_info_.size(), grtp_path_.c_str());
            }
        }
    }
    else {
        RCLCPP_WARN(node_->get_logger(), "Tool frame not set");
    }
}

void RobotHandle::Impl::initDatabase() {
    YAML::Node root = YAML::LoadFile(grtp_path_);
    if (root["database_buffer_size"]) {
        int bs = root["database_buffer_size"].as<int>();
        DataBase::init(bs, joint_names_, controller_update_period_);
        return;
    }
}

void RobotHandle::Impl::loadCartesianLimits() {
    YAML::Node root = YAML::LoadFile(grtp_path_);
    YAML::Node limits = root["cartesian_limits"];
    if (limits && limits.IsMap()) {
        cartisian_limits_max_trans_vel_ = limits["max_trans_vel"].as<double>(cartisian_limits_max_trans_vel_);
        cartisian_limits_max_trans_acc_ = limits["max_trans_acc"].as<double>(cartisian_limits_max_trans_acc_);
        cartisian_limits_max_trans_dec_ = limits["max_trans_dec"].as<double>(cartisian_limits_max_trans_dec_);
        cartisian_limits_max_rot_vel_ = limits["max_rot_vel"].as<double>(cartisian_limits_max_rot_vel_);
        RCLCPP_INFO(node_->get_logger(), "Loaded cartesian_limits from file %s.", grtp_path_.c_str());
    } else {
        RCLCPP_WARN(node_->get_logger(), "cartesian_limits not found in %s.", grtp_path_.c_str());
    }
}

void RobotHandle::Impl::loadDragParams() {
    // Only load drag_params from the YAML file specified by grtp_path cached in
    drag_params_.clear();

    YAML::Node root = YAML::LoadFile(grtp_path_);
    YAML::Node drag_node = root["drag_params"];
    if (drag_node) {
        Eigen::VectorXd eD = Eigen::VectorXd::Zero(joint_names_.size());
        Eigen::VectorXd eM = Eigen::VectorXd::Zero(joint_names_.size());
        for (const auto &pair : drag_node) {
            std::string jn = pair.first.as<std::string>();
            auto idx = std::find(joint_names_.begin(), joint_names_.end(), jn) - joint_names_.begin();
            if(idx == joint_names_.size()) {continue;}
            const auto &vals = pair.second;
            double D = vals["D"].as<double>(0.0);
            double M = vals["M"].as<double>(0.0);
            eD(idx) = D;
            eM(idx) = M;
        }
        drag_params_[DragParamEnum::D] = eD;
        drag_params_[DragParamEnum::M] = eM;

        if (!drag_params_.empty()) {
            RCLCPP_INFO(node_->get_logger(), "Loaded %zu drag_params entries from file %s.", drag_params_.size(), grtp_path_.c_str());
        } else {
            RCLCPP_WARN(node_->get_logger(), "No drag_params entries found in %s.", grtp_path_.c_str());
        }
    } else {
        RCLCPP_WARN(node_->get_logger(), "drag_params not found in %s.", grtp_path_.c_str());
    }

}

void RobotHandle::Impl::createROS2Source() {
    YAML::Node root = YAML::LoadFile(grtp_path_);
    YAML::Node controllers_name = root["controllers_name"];
    std::string position_controller_name = controllers_name["position_controller"].as<std::string>();
    std::string torque_controller_name = controllers_name["torque_controller"].as<std::string>();
    std::string cia402_controller_name = controllers_name["cia402_controller"].as<std::string>();
    std::string io_controller_name = controllers_name["io_controller"].as<std::string>();


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
    YAML::Node root = YAML::LoadFile(grtp_path_);
    YAML::Node io_group_name = root["io_group_name"];
    io_input_groups_name_ =  io_group_name["input"].as<std::vector<std::string>>();
    io_output_groups_name_ =  io_group_name["output"].as<std::vector<std::string>>();
    loadIOInterfacesNameFromService();
}

void RobotHandle::Impl::loadIOInterfacesNameFromService()
{
    io_group_command_interfaces_.clear();
    io_group_monitorable_state_interfaces_.clear();

    for (const auto &group : io_input_groups_name_) {
        io_group_command_interfaces_[group] = {};
        io_group_monitorable_state_interfaces_[group] = {};
    }
    for (const auto &group : io_output_groups_name_) {
        io_group_command_interfaces_[group] = {};
        io_group_monitorable_state_interfaces_[group] = {};
    }

    auto client = node_->create_client<controller_manager_msgs::srv::ListHardwareInterfaces>(
        "/controller_manager/list_hardware_interfaces");
    if (!client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_WARN(node_->get_logger(), "Service /controller_manager/list_hardware_interfaces is unavailable.");
        return;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::ListHardwareInterfaces::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        RCLCPP_WARN(node_->get_logger(), "Calling /controller_manager/list_hardware_interfaces timed out or failed.");
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
        auto group_it = io_group_command_interfaces_.find(group);
        if (group_it == io_group_command_interfaces_.end()) {
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
        auto group_it = io_group_monitorable_state_interfaces_.find(group);
        if (group_it == io_group_monitorable_state_interfaces_.end()) {
            continue;
        }
        group_it->second.insert(interface_name);
    }
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

DriverState RobotHandle::Impl::getDriverState(uint16_t status_word)
{
    if ((status_word & 0b01001111) == 0b00000000) {
        return DriverState::STATE_NOT_READY_TO_SWITCH_ON;
    } else if ((status_word & 0b01001111) == 0b01000000) {
        return DriverState::STATE_SWITCH_ON_DISABLED;
    } else if ((status_word & 0b01101111) == 0b00100001) {
        return DriverState::STATE_READY_TO_SWITCH_ON;
    } else if ((status_word & 0b01101111) == 0b00100011) {
        return DriverState::STATE_SWITCH_ON;
    } else if ((status_word & 0b01101111) == 0b00100111) {
        return DriverState::STATE_OPERATION_ENABLED;
    } else if ((status_word & 0b01101111) == 0b00000111) {
        return DriverState::STATE_QUICK_STOP_ACTIVE;
    } else if ((status_word & 0b01001111) == 0b00001111) {
        return DriverState::STATE_FAULT_REACTION_ACTIVE;
    } else if ((status_word & 0b01001111) == 0b00001000) {
        return DriverState::STATE_FAULT;
    }
    return DriverState::STATE_UNDEFINED;
}

uint16_t RobotHandle::Impl::ciA402Transition(DriverState state, uint16_t control_word)
{
    switch (state) {
    case DriverState::STATE_START:                     // -> STATE_NOT_READY_TO_SWITCH_ON (automatic)
        return control_word;
    case DriverState::STATE_NOT_READY_TO_SWITCH_ON:    // -> STATE_SWITCH_ON_DISABLED (automatic)
        return control_word;
    case DriverState::STATE_SWITCH_ON_DISABLED:        // -> STATE_READY_TO_SWITCH_ON
        return (control_word & 0b01111110) | 0b00000110;
    case DriverState::STATE_READY_TO_SWITCH_ON:        // -> STATE_SWITCH_ON
        return (control_word & 0b01110111) | 0b00000111;
    case DriverState::STATE_SWITCH_ON:                 // -> STATE_OPERATION_ENABLED
        return (control_word & 0b01111111) | 0b00001111;
    case DriverState::STATE_OPERATION_ENABLED:         // -> GOOD
        return control_word | 0b00011111;
    case DriverState::STATE_QUICK_STOP_ACTIVE:         // -> STATE_OPERATION_ENABLED
        return (control_word & 0b01111111) | 0b00001111;
    case DriverState::STATE_FAULT_REACTION_ACTIVE:     // -> STATE_FAULT (automatic)
        return control_word;
    case DriverState::STATE_FAULT:                     // -> STATE_SWITCH_ON_DISABLED
        return 0;
    default:
        break;
    }
    return control_word;
}

void RobotHandle::Impl::sendDriverControlWord(const uint16_t &cw)
{
    sendDriverControlMessage(CONTROL_WORD, cw);
}

void RobotHandle::Impl::switchDriverMode(const int8_t &mode)
{
    switch(mode){
    case 8:
        moveJointByAbsPosition(current_joint_position_, 1.0);break;
    case 10:{
        auto t_opt = DynamicPlugin::instance().currentPoseStableTorque(current_joint_position_);
        if(!t_opt.has_value()) {
            throw(std::runtime_error("Calculating rnea failed! Unable to switch to effort mode!"));
        }
        else {
            setJointTorque(t_opt.value());
        }
        break;
    }

    }
    sendDriverControlMessage(MODE, mode);
}

template<typename T>
void RobotHandle::Impl::sendDriverControlMessage(const std::string &name, const T &val)
{
    control_msgs::msg::DynamicInterfaceGroupValues msg;
    control_msgs::msg::InterfaceValue content;
    content.interface_names.push_back(name);
    content.values.push_back(val);
    for(const auto& joint_name : joint_names_){
        msg.interface_groups.push_back(joint_name);
        msg.interface_values.push_back(content);
    }
    motor_driver_control_publisher_->publish(msg);
}

void RobotHandle::Impl::enableMotorDrive()
{
    if(std::all_of(current_joint_status_.begin(), current_joint_status_.end(),
                    [](const auto& s) { return s.second == DriverState::STATE_OPERATION_ENABLED; })) {
        driver_state_transition_timer_->cancel();
        return;
    }
    control_msgs::msg::DynamicInterfaceGroupValues msg;
    setJointTorque(current_joint_torque_);
    moveJointByAbsPosition(current_joint_position_, 1.0);
    for(const auto& joint_name: joint_names_) {
        control_msgs::msg::InterfaceValue content;
        content.interface_names.push_back(CONTROL_WORD);
        auto cw = ciA402Transition(current_joint_status_[joint_name], 0xf);
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
    for(const auto& j_name : joint_names_) {
        msg.joint_names.push_back(j_name);
        if(joint_position.find(j_name) != joint_position.end()) {
            p.positions.push_back(joint_position.at(j_name).joint_value);
        }
        else {
            p.positions.push_back(current_joint_position_.at(j_name).joint_value);
        }
        p.velocities.push_back(current_joint_position_.at(j_name).joint_info->limits->velocity * velo_ratio);
    }
    p.time_from_start = rclcpp::Duration(controller_update_period_ / 1e9, controller_update_period_);
    msg.points.push_back(p);
    moveJointByAbsPosition(msg);
}

void RobotHandle::Impl::setJointTorque(const JointsTorque &joint_torque)
{
    std_msgs::msg::Float64MultiArray msg;
    for(const auto& joint_name : joint_names_) {
        msg.data.push_back(joint_torque.at(joint_name).joint_value);
    }
    joint_torque_publisher_->publish(msg);
}

void RobotHandle::Impl::moveJointByAbsPosition(trajectory_msgs::msg::JointTrajectory &msg)
{
    if (!msg.points.empty()) {
        // convert builtin_interfaces::msg::Duration to rclcpp::Duration
        rclcpp::Duration d(msg.points.back().time_from_start);
        expected_end_time_ = node_->get_clock()->now() + d;
        has_expected_end_time_ = true;
    } else {
        has_expected_end_time_ = false;
    }
    is_running_ = true;
    stationary_count_ = 0;
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

    DynamicPlugin::init();
    auto update_rate = AcquireParam<int32_t>("/controller_manager", "update_rate").value();
    impl_->controller_update_period_ = 1e9 / update_rate;

    impl_->loadURDFFrames();
    impl_->setupKinematicsJoints();
    impl_->initDatabase();
    impl_->loadCartesianLimits();
    impl_->loadDragParams();
    impl_->createROS2Source();
    impl_->loadIOModulesName();
    switchToCSP();
}

RobotHandle::~RobotHandle() = default;


const urdf::Model& RobotHandle::getURDFModel() const {
    return impl_->model_;
}

const KDL::Chain& RobotHandle::getKDLChain() const {
    return impl_->kdl_chain_;
}

const JointsPosition& RobotHandle::getCurrentJointPosition() const {
    return impl_->current_joint_position_;
}

const JointsVelocity &RobotHandle::getCurrentJointVelocity() const {
    return impl_->current_joint_velocity_;
}

const JointsTorque &RobotHandle::getCurrentJointTorque() const
{
    return impl_->current_joint_torque_;
}

const JointsTorque &RobotHandle::getCurrentJointEstimatedExternalTorque() const {
    return impl_->current_joint_estimated_external_torque_;
}

// helper function

size_t RobotHandle::getJointNums() const {
    return impl_->joint_names_.size();
}

const double& RobotHandle::getJointVelocityLimit(const std::string& j_name) const {
    return impl_->current_joint_position_.at(j_name).joint_info->limits->velocity;
}

const double& RobotHandle::getJointLowerLimit(const std::string& j_name) const {
    return impl_->current_joint_position_.at(j_name).joint_info->limits->lower;
}

const double& RobotHandle::getJointUpperLimit(const std::string& j_name) const {
    return impl_->current_joint_position_.at(j_name).joint_info->limits->upper;
}

// Strictly follow kinematic chain' order
const std::vector<std::string>& RobotHandle::getJointsName() const {
    return impl_->joint_names_;
}

auto RobotHandle::getTime() -> rclcpp::Time {
    return impl_->node_->get_clock()->now();
}

auto RobotHandle::getCartesianLimitsMaxTransVel() const -> const double& {
    return impl_->cartisian_limits_max_trans_vel_;
}

auto RobotHandle::getCartesianLimitsMaxTransAcc() const -> const double& {
    return impl_->cartisian_limits_max_trans_acc_;
}

auto RobotHandle::getCartesianLimitsMaxTransDec() const -> const double& {
    return impl_->cartisian_limits_max_trans_dec_;
}

auto RobotHandle::getCartesianLimitsMaxRotVel() const -> const double& {
    return impl_->cartisian_limits_max_rot_vel_;
}

const uint64_t& RobotHandle::getControllerUpdatePeriod() const {
    return impl_->controller_update_period_;
}

const std::string& RobotHandle::getRobotArmBaseLinkName() const {
    return impl_->robot_arm_base_link_name_;
}

const std::string& RobotHandle::getRobotArmEndLinkName() const {
    return impl_->robot_arm_end_link_name_;
}

const ToolInfo& RobotHandle::getRobotArmToolInfo() const {
    return impl_->robot_arm_tool_info_;
}

const DragParams& RobotHandle::getDragParams() const {
    return impl_->drag_params_;
}

void RobotHandle::moveJointByVelcoity(const JointsVelocity& joint_velocity) {
    RCLCPP_ERROR(impl_->node_->get_logger(), "Deprecated interface!");
}

void RobotHandle::moveJointByAbsPosition(const JointsPosition &joint_position, double velo_ratio)
{
    impl_->moveJointByAbsPosition(joint_position, velo_ratio);
}

void RobotHandle::moveJointByAbsPosition(trajectory_msgs::msg::JointTrajectory &joint_position)
{
    impl_->moveJointByAbsPosition(joint_position);
}

void RobotHandle::setJointTorque(const JointsTorque &joint_torque)
{
    impl_->setJointTorque(joint_torque);
}

void RobotHandle::deleteToolFrame(const std::string& tool_name) {
    impl_->robot_arm_tool_info_.erase(tool_name);
}

void RobotHandle::addToolFrame(const std::string& tool_name, const KDL::Frame& frame) {
    impl_->robot_arm_tool_info_[tool_name] = frame;
}

void RobotHandle::setCurrentToolFrame(const std::string& tool_name) {
    impl_->current_tool_frame_ = tool_name;
}

const bool& RobotHandle::isToolFrameSet() const {
    return impl_->tool_frame_set_;
}

const std::string& RobotHandle::getCurrentToolFrame() const {
    return impl_->current_tool_frame_;
}

void RobotHandle::Impl::jointStateCallback(const sensor_msgs::msg::JointState &msg)
{
    auto& data_base = DataBase::instance();
    auto observer_ready = DynamicPlugin::instance().isReady();
    for(size_t i = 0; i < msg.name.size(); ++i) {
        current_joint_position_[msg.name[i]].joint_value = msg.position[i];
        current_joint_velocity_[msg.name[i]].joint_value = msg.velocity[i];
        current_joint_torque_[msg.name[i]].joint_value = msg.effort[i];
        data_base.appendData(DataTypeEnum::POSITION, msg.name[i], msg.position[i]);
        data_base.appendData(DataTypeEnum::VELOCITY, msg.name[i], msg.velocity[i]);
        data_base.appendData(DataTypeEnum::TORQUE, msg.name[i], msg.effort[i]);
    }
    if(observer_ready) {
        // auto est_t = DynamicPlugin::instance().firstOrderMomentum(msg.position, msg.velocity, msg.effort);
        // for(size_t i = 0; i < msg.name.size(); ++i) {
        //     data_base.appendData(DataTypeEnum::ESTIMATED_TORQUE, msg.name[i], est_t.first[i] + joint_torque_offset_[msg.name[i]].joint_value);
        //     data_base.appendData(DataTypeEnum::ESTIMATED_CARTESIAN_TORQUE, msg.name[i], est_t.second[i]);
        //     current_joint_estimated_external_torque_[msg.name[i]].joint_value = est_t.first[i];
        // }
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
        stationary_count_ = 0;
        is_running_ = true;
    } else {
        stationary_count_++;
        if (stationary_count_ >= stationary_required_count_) {
            // only clear running if we are past the expected end time (with slack)
            if (has_expected_end_time_) {
                auto now = node_->get_clock()->now();
                if (now >= expected_end_time_) {
                    is_running_ = false;
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
    for(size_t i = 0; i < msg.interface_groups.size(); ++i) {
        const auto& vals = msg.interface_values[i];
        const auto& joint_name = msg.interface_groups[i];
        if(current_joint_position_.find(joint_name) == current_joint_position_.end()) {
            RCLCPP_WARN(node_->get_logger(), "Getting a driver status which does not exist in urdf!");
            continue;
        }
        for(size_t j = 0; j < vals.interface_names.size(); ++j) {
            const auto& if_name = vals.interface_names[j];
            if(if_name == MODE){
                current_joint_mode_[joint_name] = (int8_t)vals.values[j];
            }
            else if(if_name == STATUS_WORD) {
                current_joint_status_[joint_name] = getDriverState(vals.values[j]);
            }
            else {
                RCLCPP_WARN(node_->get_logger(), "Getting undefined interface from driver!");
            }
        }
    }

    std::vector<MotorStatusCallback> callbacks;
    {
        std::lock_guard<std::mutex> lock(motor_status_callbacks_mutex_);
        callbacks.reserve(motor_status_callbacks_.size());
        for (const auto& [_, cb] : motor_status_callbacks_) {
            callbacks.push_back(cb);
        }
    }

    for(const auto& sc : callbacks) {
        sc(current_joint_status_);
    }
}

void RobotHandle::Impl::ioStatusCallback(const control_msgs::msg::DynamicInterfaceGroupValues &msg)
{
    constexpr double kBoolTolerance = 1e-6;

    io_status_.clear();
    for(size_t i = 0; i < msg.interface_groups.size(); ++i) {
        const auto& vals = msg.interface_values[i];
        const auto& module_name = msg.interface_groups[i];
        if (std::find(io_input_groups_name_.begin(), io_input_groups_name_.end(), module_name) != io_input_groups_name_.end() ||
            std::find(io_output_groups_name_.begin(), io_output_groups_name_.end(), module_name) != io_output_groups_name_.end()) {
            io_status_.push_back({module_name,{}});
            for(size_t i = 0; i < vals.interface_names.size(); ++i) {
                const double raw_value = vals.values[i];
                IOValue io_value = std::nullopt;

                if (std::isfinite(raw_value)) {
                    if (std::abs(raw_value) <= kBoolTolerance) {
                        io_value = false;
                    } else if (std::abs(raw_value - 1.0) <= kBoolTolerance) {
                        io_value = true;
                    }
                }

                io_status_.back().second.push_back({vals.interface_names[i], io_value});
            }
        }
        else {
            RCLCPP_WARN(node_->get_logger(), "Detected IO module have not been registered");
        }
    }

    std::vector<IOStatusCallback> callbacks;
    {
        std::lock_guard<std::mutex> lock(io_status_callbacks_mutex_);
        callbacks.reserve(io_status_callbacks_.size());
        for (const auto& [_, cb] : io_status_callbacks_) {
            callbacks.push_back(cb);
        }
    }

    for(const auto& sc : callbacks) {
        sc(io_status_);
    }
}

size_t RobotHandle::Impl::registerMotorStatusCallback(MotorStatusCallback cb)
{
    std::lock_guard<std::mutex> lock(motor_status_callbacks_mutex_);
    const size_t callback_id = motor_status_callback_next_id_++;
    motor_status_callbacks_[callback_id] = std::move(cb);
    return callback_id;
}

void RobotHandle::Impl::unregisterMotorStatusCallback(size_t callback_id)
{
    std::lock_guard<std::mutex> lock(motor_status_callbacks_mutex_);
    motor_status_callbacks_.erase(callback_id);
}

size_t RobotHandle::Impl::registerIOStatusCallback(IOStatusCallback cb)
{
    std::lock_guard<std::mutex> lock(io_status_callbacks_mutex_);
    const size_t callback_id = io_status_callback_next_id_++;
    io_status_callbacks_[callback_id] = std::move(cb);
    return callback_id;
}

void RobotHandle::Impl::unregisterIOStatusCallback(size_t callback_id)
{
    std::lock_guard<std::mutex> lock(io_status_callbacks_mutex_);
    io_status_callbacks_.erase(callback_id);
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
    impl_->is_running_ = is_running;
}

const bool &RobotHandle::isRunning() const
{
    return impl_->is_running_;
}

void RobotHandle::setJointTorqueOffset(const std::string& joint_name, double v){
    impl_->joint_torque_offset_[joint_name].joint_value = v;
}

size_t RobotHandle::registerMotorStatusCallback(MotorStatusCallback cb)
{
    return impl_->registerMotorStatusCallback(std::move(cb));
}

void RobotHandle::unregisterMotorStatusCallback(size_t callback_id)
{
    impl_->unregisterMotorStatusCallback(callback_id);
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
    // ControllerSwitcher::instance().switchToJTC();
    impl_->switchDriverMode(8);
}

void RobotHandle::switchToCST()
{
    // ControllerSwitcher::instance().switchToEffect();
    impl_->switchDriverMode(10);
}

const std::vector<std::string>& RobotHandle::getIOInputGroupsName() const
{
    return impl_->io_input_groups_name_;
}

const std::vector<std::string>& RobotHandle::getIOOutputGroupsName() const
{
    return impl_->io_output_groups_name_;
}

const std::vector<std::string>& RobotHandle::getIOInterfacesName(const std::string &module_name) const
{
    static const std::vector<std::string> kEmpty;
    auto it = impl_->io_group_command_interfaces_.find(module_name);
    if (it == impl_->io_group_command_interfaces_.end()) {
        return kEmpty;
    }
    return it->second;
}

bool RobotHandle::isIOMonitorable(const std::string &module_name, const std::string &interface_name) const
{
    auto it = impl_->io_group_monitorable_state_interfaces_.find(module_name);
    if (it == impl_->io_group_monitorable_state_interfaces_.end()) {
        return false;
    }
    return it->second.find(interface_name) != it->second.end();
}

size_t RobotHandle::registerIOStatusCallback(IOStatusCallback cb)
{
    return impl_->registerIOStatusCallback(std::move(cb));
}

void RobotHandle::unregisterIOStatusCallback(size_t callback_id)
{
    impl_->unregisterIOStatusCallback(callback_id);
}

void RobotHandle::setIOState(const std::string &module_name, const std::string &interface_name, bool target_state)
{
    impl_->setIOState(module_name, interface_name, target_state);
}
