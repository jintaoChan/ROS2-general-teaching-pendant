#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <unordered_map>
#include <Eigen/Eigen>
#include "singleton.hpp"
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>

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

class DataBase;
template<typename T> class Singleton;

enum class DriverState
{
    STATE_UNDEFINED = 0,
    STATE_START = 1,
    STATE_NOT_READY_TO_SWITCH_ON,
    STATE_SWITCH_ON_DISABLED,
    STATE_READY_TO_SWITCH_ON,
    STATE_SWITCH_ON,
    STATE_OPERATION_ENABLED,
    STATE_QUICK_STOP_ACTIVE,
    STATE_FAULT_REACTION_ACTIVE,
    STATE_FAULT
};

enum class ControlCoordinateSystemType {
    Base = 0,
    Tool,
    EndEffector
};

struct Joint {
    std::shared_ptr<const urdf::Joint> joint_info;
    double joint_value = 0.0;
};

using JointsPosition = std::unordered_map<std::string, Joint>;
using JointsVelocity = std::unordered_map<std::string, Joint>;
using JointsAcceleration = std::unordered_map<std::string, Joint>;
using JointsTorque = std::unordered_map<std::string, Joint>;
using JointsMode = std::unordered_map<std::string, int8_t>;
using JointsStatus = std::unordered_map<std::string, DriverState>;
using IOValue = std::optional<bool>;
using IOStatus = std::vector<std::pair<std::string, std::vector<std::pair<std::string, IOValue>>>>; //<module_name, <interface_name, val>>
using ToolInfo = std::unordered_map<std::string, KDL::Frame>;

enum class MoveTypeEnum : char {
    POSE = 0,
    JOINT
};

struct TargetPointInfo {
    MoveTypeEnum MoveType;
    JointsPosition JointValues;
    KDL::Frame Pose;
    double VelocityRatio{0.1};
};
using MovePointInfo  = std::unordered_map<std::string, TargetPointInfo>;


// Drag parameters per joint (simple storage for admittance / momentum observer tuning)
enum class DragParamEnum : uint8_t{
    D = 0,
    M,
};

using DragParams = std::unordered_map<DragParamEnum, Eigen::VectorXd>;

using MotorStatusCallback = std::function<void(JointsStatus)>;
using IOStatusCallback = std::function<void(IOStatus)>;

class RobotHandle : public Singleton<RobotHandle> {
    friend class Singleton<RobotHandle>;

public:
    explicit RobotHandle(const std::shared_ptr<rclcpp::Node>& node);
    ~RobotHandle();

    // accessors (same as before)
    const urdf::Model& getURDFModel() const;
    const KDL::Chain& getKDLChain() const;
    const JointsPosition& getCurrentJointPosition() const;
    const JointsVelocity& getCurrentJointVelocity() const;
    const JointsTorque& getCurrentJointTorque() const;
    const JointsTorque& getCurrentJointEstimatedExternalTorque() const;
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

    const uint64_t& getControllerUpdatePeriod() const;

    const std::string& getRobotArmBaseLinkName() const;
    const std::string& getRobotArmEndLinkName() const;
    const ToolInfo& getRobotArmToolInfo() const;
    const DragParams& getDragParams() const;

    void moveJointByVelcoity(const JointsVelocity& joint_velocity);
    void moveJointByAbsPosition(const JointsPosition& joint_position, double velo_ratio);
    void moveJointByAbsPosition(trajectory_msgs::msg::JointTrajectory& joint_position);
    void setJointTorque(const JointsTorque& joint_torque);

    void deleteToolFrame(const std::string& tool_name);
    void addToolFrame(const std::string& tool_name, const KDL::Frame& frame);
    void setCurrentToolFrame(const std::string& tool_name);

    const bool& isToolFrameSet() const;
    const std::string& getCurrentToolFrame() const;

    void setIsRunning(bool is_running);
    const bool& isRunning() const;
    void setJointTorqueOffset(const std::string& joint_name, double v);

    //CiA402 control
    void registerMotorStatusCallback(MotorStatusCallback cb);
    void disableMotorDrive();
    void clearFault();
    void enableMotorDrive();
    void switchToCSP();
    void switchToCST();

    //io
    const std::vector<std::string>& getIOInputGroupsName() const;
    const std::vector<std::string>& getIOOutputGroupsName() const;
    const std::vector<std::string>& getIOInterfacesName(const std::string& module_name) const;
    bool isIOMonitorable(const std::string& module_name, const std::string& interface_name) const;
    void registerIOStatusCallback(IOStatusCallback cb);
    void setIOState(const std::string& module_name, const std::string& interface_name, bool target_state);


private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

