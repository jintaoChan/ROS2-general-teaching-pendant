#include "robot_handle.h"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>
#include <optional>
#include "database.h"
#include "singleton.hpp"
#include "functional.hpp"
#include "controller_switcher.h"

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class RobotHandle::Impl {
public:
    rclcpp::Node::SharedPtr node_;
    std::optional<rclcpp::Logger> logger_;
    std::shared_ptr<RobotHandle> singleton_;
    urdf::Model model_;
    std::shared_ptr<::urdf::ModelInterface> urdf_tree_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr controller_state_subscription_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr move_command_sender_;

    int32_t controller_update_period_; //ms
    std::vector<std::string> joint_names_;
    JointsPosition current_joint_position_;
    std::string robot_arm_base_link_name_;
    std::string robot_arm_end_link_name_;
    ToolInfo robot_arm_tool_info_;
    bool tool_frame_set_;
    std::string current_tool_frame_;

    double cartisian_limits_max_trans_vel_;
    double cartisian_limits_max_trans_acc_;
    double cartisian_limits_max_trans_dec_;
    double cartisian_limits_max_rot_vel_;

    control_msgs::msg::JointTrajectoryControllerState controller_state_msg_;

    void jointStateCallback(const sensor_msgs::msg::JointState &msg);
    void controllerStateCallback(const control_msgs::msg::JointTrajectoryControllerState &msg);

};


RobotHandle::RobotHandle(const std::shared_ptr<rclcpp::Node>& node)
    : impl_(std::make_unique<Impl>())
{
    impl_->node_ = node;
    impl_->logger_ = node->get_logger();

    auto urdf_string = AcquireParam<std::string>("/robot_state_publisher", "robot_description").value();
    if (!impl_->model_.initString(urdf_string)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to parse URDF model.");
        return;
    }
    impl_->urdf_tree_ = urdf::parseURDF(urdf_string);
    RCLCPP_INFO(node->get_logger(), "Successfully parsed URDF model.");

    std::vector<urdf::LinkSharedPtr> link_ptrs;
    impl_->model_.getLinks(link_ptrs);
    std::vector<std::string> available_names;
    for(const auto& l: link_ptrs) {
        available_names.push_back(l->name);
        RCLCPP_INFO(node->get_logger(), "Got frame %s: ", l->name.c_str());
    }
    getFrameName(impl_->robot_arm_base_link_name_, "coordinate_system.base_frame", available_names, available_names.front());
    getFrameName(impl_->robot_arm_end_link_name_, "coordinate_system.end_effector_frame", available_names, available_names.back());
    getToolFrameInfo(impl_->robot_arm_tool_info_, "coordinate_system.tool_frame");

    auto update_rate = AcquireParam<int32_t>("/controller_manager", "update_rate").value();
    impl_->controller_update_period_ = 1e3 / update_rate;
    kdl_parser::treeFromUrdfModel(impl_->model_, impl_->kdl_tree_);
    impl_->kdl_tree_.getChain(impl_->robot_arm_base_link_name_, impl_->robot_arm_end_link_name_, impl_->kdl_chain_);
    for (const auto& seg : impl_->kdl_chain_.segments) {
        auto joint = seg.getJoint();
        if(joint.getType() == KDL::Joint::JointType::Fixed) {continue;}
        Joint j{impl_->model_.getJoint(joint.getName()), 0};
        impl_->joint_names_.push_back(joint.getName());
        impl_->current_joint_position_[joint.getName()] = j;
    }
    auto buffer_size = AcquireParam<int32_t>("/controller_manager", "database_buffer_size");
    if(buffer_size.has_value()){
        DataBase::init(buffer_size.value(), impl_->joint_names_, impl_->controller_update_period_);
    }
    else {
        DataBase::init(100 * update_rate, impl_->joint_names_, impl_->controller_update_period_);
    }

    impl_->cartisian_limits_max_trans_vel_ = AcquireParam<double>("/controller_manager", "cartesian_limits.max_trans_vel").value();
    impl_->cartisian_limits_max_trans_acc_ = AcquireParam<double>("/controller_manager",  "cartesian_limits.max_trans_acc").value();
    impl_->cartisian_limits_max_trans_dec_ = AcquireParam<double>("/controller_manager",  "cartesian_limits.max_trans_dec").value();
    impl_->cartisian_limits_max_rot_vel_ = AcquireParam<double>("/controller_manager",  "cartesian_limits.max_rot_vel").value();

    impl_->joint_state_subscription_ = impl_->node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&RobotHandle::Impl::jointStateCallback, *this->impl_, std::placeholders::_1));
    impl_->controller_state_subscription_ = impl_->node_->create_subscription<control_msgs::msg::JointTrajectoryControllerState>("/trajectory_controller/controller_state", 10, std::bind(&RobotHandle::Impl::controllerStateCallback, *this->impl_, std::placeholders::_1));
    impl_->move_command_sender_ = impl_->node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("trajectory_controller/joint_trajectory", 10);

}

RobotHandle::~RobotHandle() = default;


const urdf::Model& RobotHandle::getURDFModel() const {
    return impl_->model_;
}

const std::shared_ptr<::urdf::ModelInterface>& RobotHandle::getURDFTree() const {
    return impl_->urdf_tree_;
}

const KDL::Chain& RobotHandle::getKDLChain() const {
    return impl_->kdl_chain_;
}

const JointsPosition& RobotHandle::getCurrentJointPosition() const {
    return impl_->current_joint_position_;
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

const int32_t& RobotHandle::getControllerUpdatePeriod() const {
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

void RobotHandle::moveJointByVelcoity(const JointsVelocity& joint_velocity) {
    RCLCPP_ERROR(impl_->node_->get_logger(), "Deprecated interface!");
}

void RobotHandle::moveJointByAbsPosition(const JointsPosition& joint_position, double velo_ratio) {
    // ControllerSwitcher::instance().switchToControlPad();
    trajectory_msgs::msg::JointTrajectory msg;
    trajectory_msgs::msg::JointTrajectoryPoint p;
    for(const auto& j_name : impl_->joint_names_) {
        msg.joint_names.push_back(j_name);
        if(joint_position.find(j_name) != joint_position.end()) {
            p.positions.push_back(joint_position.at(j_name).joint_value);
        }
        else {
            p.positions.push_back(impl_->current_joint_position_.at(j_name).joint_value);
        }
        p.velocities.push_back(impl_->current_joint_position_.at(j_name).joint_info->limits->velocity * velo_ratio);
    }
    p.time_from_start = rclcpp::Duration(impl_->controller_update_period_ / 1000, impl_->controller_update_period_ * 1000000);
    msg.points.push_back(p);
    moveJointByAbsPosition(msg);
}

void RobotHandle::moveJointByAbsPosition(trajectory_msgs::msg::JointTrajectory &msg)
{
    msg.header.stamp = getTime();
    impl_->move_command_sender_->publish(msg);
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
    for(size_t i = 0; i < msg.name.size(); ++i) {
        current_joint_position_[msg.name[i]].joint_value = msg.position[i];
        data_base.appendData(DataTypeEnum::POSITION, msg.name[i], msg.position[i]);
        data_base.appendData(DataTypeEnum::VELOCITY, msg.name[i], msg.velocity[i]);
        data_base.appendData(DataTypeEnum::TORQUE, msg.name[i], msg.effort[i]);
    }
}

void RobotHandle::Impl::controllerStateCallback(const control_msgs::msg::JointTrajectoryControllerState &msg)
{
    controller_state_msg_ = msg;
}

//warpper function
KDL::Frame RobotHandle::GetFixedTransform(const KDL::Chain& chain)
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
void RobotHandle::getToolFrameInfo(ToolInfo& res, const std::string& param_name) {
    urdf::Model urdf_model;
    KDL::Tree kdl_tree;
    KDL::Chain chain;
    auto param = AcquireParam<std::vector<std::string>>("/controller_manager", param_name);
    auto urdf_param = AcquireParam<std::string>("/robot_state_publisher", "robot_description").value();
    urdf_model.initString(urdf_param);
    kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree);
    auto all_links = kdl_tree.getSegments();
    if(param.has_value()) {
        for(auto v: param.value()) {
            if(!all_links.count(v)) {
                RCLCPP_WARN(impl_->node_->get_logger(), "Tool frame %s do not exist in urdf! Ignore this tool!", v.c_str());
                continue;
            }
            kdl_tree.getChain(impl_->robot_arm_end_link_name_, v, chain);
            res[v] = GetFixedTransform(chain);
        }
        impl_->tool_frame_set_ = true;
        impl_->current_tool_frame_ = res.begin()->first;
    }
    else {
        res.clear();
    }
}

void RobotHandle::getFrameName(std::string& res, const std::string& param_name, const std::vector<std::string>& available_list, const std::string& or_value) {
    auto param = AcquireParam<std::string>("/controller_manager", param_name);
    if(param.has_value()) {
        res = param.value();
        if(std::find(available_list.begin(), available_list.end(), res) == available_list.end()) {
            RCLCPP_WARN(impl_->node_->get_logger(), "Frame %s do not exist in urdf!", res.c_str());
            res = available_list.front();
        }
    }
    else {
        res = or_value;
    }
}

