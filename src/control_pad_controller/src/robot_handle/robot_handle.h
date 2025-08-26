#pragma once

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>

#include "singleton.hpp"
#include "functional.hpp"
#include "controller_switcher.h"

enum class ControlCoordinateSystemType {
    Base = 0,
    Tool,
    EndEffector
};

enum class MoveTypeEnum : char {
    POSE = 0, //position + posture
    POSITION,
    POSTURE,
    JOINT
};

struct TargetPointInfo {
    MoveTypeEnum MoveType;
    std::vector<double> Values;
    std::vector<std::string> JointNames;
};

//std::unordered_map<GroupName, TargetPointInfo>
using MovePointInfo  = std::unordered_map<std::string, TargetPointInfo>;

//std::unordered_map<PointName, MovePointInfo>
using MovePointInfos = std::unordered_map<std::string, MovePointInfo>;

struct PointGroupTask {
    int Times;
    std::vector<MovePointInfo> Points;
};

using TaskUnion = std::variant<MovePointInfo, PointGroupTask>;

struct MoveTask {
    std::string PointName;
    TaskUnion PointInfos;
};
using MoveTasks = std::vector<MoveTask>;

struct Joints {
    moveit::core::JointModel::JointType joint_type{moveit::core::JointModel::JointType::UNKNOWN};
    double joint_value{0};
};

using JointsPosition = std::unordered_map<std::string, Joints>;
using JointsVelocity = std::unordered_map<std::string, Joints>;
using JointsAcceleration = std::unordered_map<std::string, Joints>;
using ToolInfo = std::unordered_map<std::string, KDL::Frame>;

class RobotHandle : public Singleton<RobotHandle>{
    friend class Singleton<RobotHandle>;
public:
    RobotHandle(const rclcpp::Node::SharedPtr& node):
    node_(node),
    robot_model_loader_(node_),
    logger_(node_->get_logger()),
    robot_model_ptr_(robot_model_loader_.getModel()),
    robot_state_(std::make_shared<moveit::core::RobotState>(robot_model_ptr_)),
    joint_model_groups_(robot_model_ptr_->getJointModelGroups())
    {
        size_t max_joint_num = 0;
        for(const auto& group_name : getJointGroupsNames()) {
            move_group_interfaces_[group_name] = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);
            if(move_group_interfaces_[group_name]->getJoints().size() > max_joint_num) {
                max_joint_num = move_group_interfaces_[group_name]->getJoints().size();
                robot_arm_name_ = group_name;
            }
        }
        const auto& available_names = move_group_interfaces_[robot_arm_name_]->getLinkNames();
        getFrameName(robot_arm_base_link_name_, "coordinate_system.base_frame", available_names, available_names.front());
        getFrameName(robot_arm_end_link_name_, "coordinate_system.end_effector_frame", available_names, available_names.back());
        getToolFrameInfo(robot_arm_tool_info_, "coordinate_system.tool_frame");

        
        robot_frame_id_ = move_group_interfaces_[robot_arm_name_]->getPlanningFrame();
        RCLCPP_INFO(node_->get_logger(), "IK plugin manipulating: %s", robot_arm_name_.c_str());
        RCLCPP_INFO(node_->get_logger(), "Robot frame: %s", robot_frame_id_.c_str());
        
        auto update_rate = AcquireParam<int32_t>("/controller_manager", "update_rate").value();
        controller_update_period_ = 1e3 / update_rate;

        const auto& joint_models = robot_model_ptr_->getJointModels();
        for (const auto* joint_model : joint_models) {
            current_joint_position_[joint_model->getName()].joint_type = joint_model->getType();
        }
        current_joint_position_.erase("ASSUMED_FIXED_ROOT_JOINT");

        auto joint_groups_names = getJointGroupsNames();
        for(const auto& jg_name: joint_groups_names) {
            auto joint = getJointGroupJointNames(jg_name);
            for(const auto& jt : joint) {
                joint_velocity_limits_[jt].joint_value = AcquireParam<double>("/move_group", "robot_description_planning.joint_limits." + jt + ".max_velocity").value();
                joint_acceleration_limits_[jt].joint_value = AcquireParam<double>("/move_group", "robot_description_planning.joint_limits." + jt + ".max_acceleration").value();
                joint_deceleration_limits_[jt].joint_value = AcquireParam<double>("/move_group", "robot_description_planning.joint_limits." + jt + ".max_deceleration").value();
            }
        }

        cartisian_limits_max_trans_vel_ = AcquireParam<double>("/move_group", "robot_description_planning.cartesian_limits.max_trans_vel").value();
        cartisian_limits_max_trans_acc_ = AcquireParam<double>("/move_group",  "robot_description_planning.cartesian_limits.max_trans_acc").value();
        cartisian_limits_max_trans_dec_ = AcquireParam<double>("/move_group",  "robot_description_planning.cartesian_limits.max_trans_dec").value();
        cartisian_limits_max_rot_vel_ = AcquireParam<double>("/move_group",  "robot_description_planning.cartesian_limits.max_rot_vel").value();

        joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&RobotHandle::recvCallback, this, std::placeholders::_1));
        move_command_sender_ = node_->create_publisher<sensor_msgs::msg::JointState>("control_pad_controller/control_pad_move_cmd", 10);
    };    

    ~RobotHandle() {
    }
    
public:
    // Acquire names of each joint group
    auto getJointGroupsNames() const -> const std::vector<std::string>& {
        return robot_model_ptr_->getJointModelGroupNames();
    }
    // Acquire joint names in specified joint group
    auto getJointGroupJointNames(const std::string& jointGroupName) const -> const std::vector<std::string>& {
        return robot_model_ptr_->getJointModelGroup(jointGroupName)->getVariableNames();
    }
    auto getJointGroupJointPositions(const std::string& jointGroupName) const -> std::vector<double> {
        std::vector<double> jointPositions;
        robot_state_->copyJointGroupPositions(robot_model_ptr_->getJointModelGroup(jointGroupName), jointPositions);
        return jointPositions;
    }
    auto getMoveGroupInterfaces(const std::string& group_name) -> moveit::planning_interface::MoveGroupInterfacePtr& {
        return move_group_interfaces_[group_name];
    }
    auto getPresetGroupState() const -> MovePointInfos {
        MovePointInfos defaultPoseList;
        for(const auto& jointGroupName : getJointGroupsNames()) {
            const auto& stateNames = robot_model_ptr_->getJointModelGroup(jointGroupName)->getDefaultStateNames();
            for(const auto& stateName : stateNames) {
                std::map<std::string, double> jointNamesAndValues;
                robot_model_ptr_->getJointModelGroup(jointGroupName)->getVariableDefaultPositions(stateName, jointNamesAndValues);
                for(const auto& j : jointNamesAndValues) {
                    defaultPoseList[stateName][jointGroupName].JointNames.push_back(j.first);
                    defaultPoseList[stateName][jointGroupName].Values.push_back(j.second);
                }
                defaultPoseList[stateName][jointGroupName].MoveType = MoveTypeEnum::JOINT;
            }
        }
        return defaultPoseList;
    }

    const JointsPosition& getCurrentJointPosition() const {
        return current_joint_position_;
    }

    geometry_msgs::msg::Pose getCurrentEndEffectorPose() const {
        auto pose_stamped = move_group_interfaces_.at(robot_arm_name_)->getCurrentPose();
        return pose_stamped.pose;
    }

    auto getRobotArmName() const -> const std::string& {
        return robot_arm_name_;
    }

    auto getFrameID() const -> const std::string& {
        return robot_frame_id_;
    }

    auto getTime() -> rclcpp::Time {
        return node_->get_clock()->now();
    }

    auto getCartesianLimitsMaxTransVel() const -> const double& {
        return cartisian_limits_max_trans_vel_;
    }

    auto getCartesianLimitsMaxTransAcc() const -> const double& {
        return cartisian_limits_max_trans_acc_;
    }

    auto getCartesianLimitsMaxTransDec() const -> const double& {
        return cartisian_limits_max_trans_dec_;
    }

    auto getCartesianLimitsMaxRotVel() const -> const double& {
        return cartisian_limits_max_rot_vel_;
    }

    const JointsVelocity& getJointsVelocityLimits() const {
        return joint_velocity_limits_;
    }

    const JointsAcceleration& getJointsAccelerationLimits() const {
        return joint_acceleration_limits_;
    }

    const JointsAcceleration& getJointsDecelerationLimits() const {
        return joint_deceleration_limits_;
    }

    const int32_t& getControllerUpdatePeriod() const {
        return controller_update_period_;
    }

    const std::string& getRobotArmBaseLinkName() const {
        return robot_arm_base_link_name_;
    }

    const std::string& getRobotArmEndLinkName() const {
        return robot_arm_end_link_name_;
    }

    const ToolInfo& getRobotArmToolInfo() const {
        return robot_arm_tool_info_;
    }

    void moveJointByVelcoity(const JointsVelocity& joint_velocity) {
        ControllerSwitcher::instance().switchToControlPad();
        sensor_msgs::msg::JointState msg;
        for(const auto& v : joint_velocity) {
            msg.name.push_back(v.first);
            msg.velocity.push_back(v.second.joint_value);
        }
        move_command_sender_->publish(msg);
    }

    void moveJointByAbsPosition(const JointsPosition& joint_position) {
        ControllerSwitcher::instance().switchToControlPad();
        sensor_msgs::msg::JointState msg;
        for(const auto& v : joint_position) {
            msg.name.push_back(v.first);
            msg.position.push_back(v.second.joint_value);
        }
        move_command_sender_->publish(msg);
    }

    void deleteToolFrame(const std::string& tool_name) {
        robot_arm_tool_info_.erase(tool_name);
    }

    void addToolFrame(const std::string& tool_name, const KDL::Frame& frame) {
        robot_arm_tool_info_[tool_name] = frame;
    }

    void setCurrentToolFrame(const std::string& tool_name) {
        current_tool_frame_ = tool_name;
    }

    const bool& isToolFrameSet() const {
        return tool_frame_set_;
    }

    const std::string& getCurrentToolFrame() const {
        return current_tool_frame_;
    }
    
private:
    void recvCallback(const sensor_msgs::msg::JointState &msg)
    {
        for(size_t i = 0; i < msg.name.size(); ++i) {
            current_joint_position_[msg.name[i]].joint_value = msg.position[i];
        }
    }

    //warpper function
    KDL::Frame GetFixedTransform(const KDL::Chain& chain)
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
    void getToolFrameInfo(ToolInfo& res, const std::string& param_name) {
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
                    RCLCPP_WARN(node_->get_logger(), "Tool frame %s do not exist in urdf! Ignore this tool!", v.c_str());
                    continue;
                }
                kdl_tree.getChain(robot_arm_end_link_name_, v, chain);
                res[v] = GetFixedTransform(chain);
            }
            tool_frame_set_ = true;
            current_tool_frame_ = res.begin()->first;
        }
        else {
            res.clear();
        }
    }

    void getFrameName(std::string& res, const std::string& param_name, const std::vector<std::string>& available_list, const std::string& or_value) {
        auto param = AcquireParam<std::string>("/controller_manager", param_name);
        if(param.has_value()) {
            res = param.value();
            if(std::find(available_list.begin(), available_list.end(), res) == available_list.end()) {
                RCLCPP_WARN(node_->get_logger(), "Frame %s do not exist in urdf!", res.c_str());
                res = available_list.front();
            }
        }
        else {
            res = or_value;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    robot_model_loader::RobotModelLoader robot_model_loader_;
    rclcpp::Logger logger_;
    moveit::core::RobotModelPtr robot_model_ptr_;
    moveit::core::RobotStatePtr robot_state_;
    const std::vector<moveit::core::JointModelGroup*>& joint_model_groups_;
    std::unordered_map<std::string, moveit::planning_interface::MoveGroupInterfacePtr> move_group_interfaces_;
    std::shared_ptr<RobotHandle> singleton_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr move_command_sender_;

    int32_t controller_update_period_; //ms
    JointsPosition current_joint_position_;
    JointsVelocity joint_velocity_limits_;
    JointsAcceleration joint_acceleration_limits_;
    JointsAcceleration joint_deceleration_limits_;
    std::string robot_arm_name_;
    std::string robot_frame_id_;
    std::string robot_arm_base_link_name_;
    std::string robot_arm_end_link_name_;
    ToolInfo robot_arm_tool_info_;
    bool tool_frame_set_;
    std::string current_tool_frame_;

    double cartisian_limits_max_trans_vel_;
    double cartisian_limits_max_trans_acc_;
    double cartisian_limits_max_trans_dec_;
    double cartisian_limits_max_rot_vel_;
};
