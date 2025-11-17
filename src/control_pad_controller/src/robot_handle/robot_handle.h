#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <variant>
#include "singleton.hpp"
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// ROS
namespace rclcpp {
class Node;
class Logger;
class Time;
}

// URDF
namespace urdf {
class Model;
class ModelInterface;
class Joint;
}

// KDL
namespace KDL {
class Tree;
class Chain;
class Frame;
}

// your own classes
class DataBase;
template<typename T> class Singleton;

// ---- Data structures unchanged ----

enum class ControlCoordinateSystemType {
    Base = 0,
    Tool,
    EndEffector
};

enum class MoveTypeEnum : char {
    POSE = 0,
    POSITION,
    POSTURE,
    JOINT
};

struct TargetPointInfo {
    MoveTypeEnum MoveType;
    std::vector<double> Values;
    std::vector<std::string> JointNames;
};

using MovePointInfo  = std::unordered_map<std::string, TargetPointInfo>;
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

struct Joint {
    std::shared_ptr<const urdf::Joint> joint_info;
    double joint_value = 0.0;
};

using JointsPosition = std::unordered_map<std::string, Joint>;
using JointsVelocity = std::unordered_map<std::string, Joint>;
using JointsAcceleration = std::unordered_map<std::string, Joint>;
using JointsTorque = std::unordered_map<std::string, Joint>;
using ToolInfo = std::unordered_map<std::string, KDL::Frame>;


// ---------------- PIMPL version of RobotHandle ----------------

class RobotHandle : public Singleton<RobotHandle> {
    friend class Singleton<RobotHandle>;

public:
    explicit RobotHandle(const std::shared_ptr<rclcpp::Node>& node);
    ~RobotHandle();

    // accessors (same as before)
    const urdf::Model& getURDFModel() const;
    const KDL::Chain& getKDLChain() const;
    const JointsPosition& getCurrentJointPosition() const;
    size_t getJointNums() const;
    const double& getJointVelocityLimit(const std::string&) const;
    const double& getJointLowerLimit(const std::string&) const;
    const double& getJointUpperLimit(const std::string&) const;
    const std::vector<std::string>& getJointsName() const;
    rclcpp::Time getTime();

    const double& getCartesianLimitsMaxTransVel() const;
    const double& getCartesianLimitsMaxTransAcc() const;
    const double& getCartesianLimitsMaxTransDec() const;
    const double& getCartesianLimitsMaxRotVel() const;

    const int32_t& getControllerUpdatePeriod() const;

    const std::string& getRobotArmBaseLinkName() const;
    const std::string& getRobotArmEndLinkName() const;
    const ToolInfo& getRobotArmToolInfo() const;

    void moveJointByVelcoity(const JointsVelocity& joint_velocity);
    void moveJointByAbsPosition(const JointsPosition& joint_position, double velo_ratio);
    void moveJointByAbsPosition(trajectory_msgs::msg::JointTrajectory& joint_position);

    void deleteToolFrame(const std::string& tool_name);
    void addToolFrame(const std::string& tool_name, const KDL::Frame& frame);
    void setCurrentToolFrame(const std::string& tool_name);

    const bool& isToolFrameSet() const;
    const std::string& getCurrentToolFrame() const;

    KDL::Frame GetFixedTransform(const KDL::Chain& chain);
    void getToolFrameInfo(ToolInfo& res, const std::string& param_name);
    void getFrameName(std::string& res, const std::string& param_name, const std::vector<std::string>& available_list, const std::string& or_value);
    void setIsRunning(bool is_running);
    const bool& isRunning() const;
    void setJointTorqueOffset(const std::string& joint_name, double v);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

