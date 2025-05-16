#pragma once

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <singleton.hpp>

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

//std::map<GroupName, TargetPointInfo>
using MovePointInfo  = std::map<std::string, TargetPointInfo>;

//std::map<PointName, MovePointInfo>
using MovePointInfos = std::map<std::string, MovePointInfo>;

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

class RobotDescription : public Singleton<RobotDescription> {
    friend class Singleton<RobotDescription>;
    
private:
    RobotDescription(const rclcpp::Node::SharedPtr& node):
    m_RobotModelLoader(node),
    m_Logger(node->get_logger()),
    m_RobotModelPtr(m_RobotModelLoader.getModel()),
    m_RobotState(new moveit::core::RobotState(m_RobotModelPtr)),
    m_JointModelGroups(m_RobotModelPtr->getJointModelGroups())
    {
        for(const auto& groupName : getJointGroupsNames()) {
            m_MoveGroupInterfaces[groupName] = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, groupName);
        }
        RCLCPP_INFO(m_Logger, "Model frame: %s", m_RobotModelPtr->getModelFrame().c_str());
    };
    
public:
    // Acquire names of each joint group
    auto getJointGroupsNames() const -> const std::vector<std::string>& {
        return m_RobotModelPtr->getJointModelGroupNames();
    }
    // Acquire joint names in specified joint group
    auto getJointGroupJointNames(const std::string& jointGroupName) const -> const std::vector<std::string>& {
        return m_RobotModelPtr->getJointModelGroup(jointGroupName)->getVariableNames();
    }
    auto getJointGroupJointPositions(const std::string& jointGroupName) const -> std::vector<double> {
        std::vector<double> jointPositions;
        m_RobotState->copyJointGroupPositions(m_RobotModelPtr->getJointModelGroup(jointGroupName), jointPositions);
        return jointPositions;
    }
    auto getMoveGroupInterfaces(const std::string& groupName) -> moveit::planning_interface::MoveGroupInterfacePtr& {
        return m_MoveGroupInterfaces[groupName];
    }
    auto getPresetGroupState() const -> MovePointInfos {
        MovePointInfos defaultPoseList;
        for(const auto& jointGroupName : getJointGroupsNames()) {
            const auto& stateNames = m_RobotModelPtr->getJointModelGroup(jointGroupName)->getDefaultStateNames();
            for(const auto& stateName : stateNames) {
                std::map<std::string, double> jointNamesAndValues;
                m_RobotModelPtr->getJointModelGroup(jointGroupName)->getVariableDefaultPositions(stateName, jointNamesAndValues);
                for(const auto& j : jointNamesAndValues) {
                    defaultPoseList[stateName][jointGroupName].JointNames.push_back(j.first);
                    defaultPoseList[stateName][jointGroupName].Values.push_back(j.second);
                }
                defaultPoseList[stateName][jointGroupName].MoveType = MoveTypeEnum::JOINT;
            }
        }
        return defaultPoseList;
    }
    
private:
    robot_model_loader::RobotModelLoader m_RobotModelLoader;
    rclcpp::Logger m_Logger;
    moveit::core::RobotModelPtr m_RobotModelPtr;
    moveit::core::RobotStatePtr m_RobotState;
    const std::vector<moveit::core::JointModelGroup*>& m_JointModelGroups;
    std::map<std::string, moveit::planning_interface::MoveGroupInterfacePtr> m_MoveGroupInterfaces;
    std::shared_ptr<RobotDescription> m_Singleton;

};
