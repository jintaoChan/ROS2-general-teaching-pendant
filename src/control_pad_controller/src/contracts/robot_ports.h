#pragma once

#include "robot_handle.h"

// Read-only robot state access for algorithm modules.
class IRobotStateProvider {
public:
    virtual ~IRobotStateProvider() = default;

    virtual const urdf::Model& getURDFModel() const = 0;
    virtual const KDL::Chain& getKDLChain() const = 0;

    virtual size_t getJointNums() const = 0;
    virtual const std::vector<std::string>& getJointsName() const = 0;

    virtual const JointsPosition& getCurrentJointPosition() const = 0;
    virtual const JointsVelocity& getCurrentJointVelocity() const = 0;
    virtual const JointsTorque& getCurrentJointTorque() const = 0;
    virtual const JointsTorque& getCurrentJointEstimatedExternalTorque() const = 0;

    virtual const double& getJointVelocityLimit(const std::string& j_name) const = 0;
    virtual const double& getJointLowerLimit(const std::string& j_name) const = 0;
    virtual const double& getJointUpperLimit(const std::string& j_name) const = 0;

    virtual const std::string& getRobotArmBaseLinkName() const = 0;
    virtual const std::string& getRobotArmEndLinkName() const = 0;
    virtual const ToolInfo& getRobotArmToolInfo() const = 0;
    virtual const DragParams& getDragParams() const = 0;

    virtual const bool& isToolFrameSet() const = 0;
    virtual const std::string& getCurrentToolFrame() const = 0;

    virtual const double& getCartesianLimitsMaxTransVel() const = 0;
    virtual const double& getCartesianLimitsMaxTransAcc() const = 0;
    virtual const double& getCartesianLimitsMaxTransDec() const = 0;
    virtual const double& getCartesianLimitsMaxRotVel() const = 0;

    virtual const uint64_t& getControllerUpdatePeriod() const = 0;
    virtual rclcpp::Time getTime() = 0;

    virtual const bool& isRunning() const = 0;

    virtual const std::vector<std::string>& getIOInputGroupsName() const = 0;
    virtual const std::vector<std::string>& getIOOutputGroupsName() const = 0;
    virtual const std::vector<std::string>& getIOInterfacesName(const std::string& module_name) const = 0;
    virtual bool isIOMonitorable(const std::string& module_name, const std::string& interface_name) const = 0;
};

// Robot command and mode switching port for algorithm modules.
class IRobotCommandPort {
public:
    virtual ~IRobotCommandPort() = default;

    virtual void moveJointByVelcoity(const JointsVelocity& joint_velocity) = 0;
    virtual void moveJointByAbsPosition(const JointsPosition& joint_position, double velo_ratio) = 0;
    virtual void moveJointByAbsPosition(trajectory_msgs::msg::JointTrajectory& joint_position) = 0;
    virtual void setJointTorque(const JointsTorque& joint_torque) = 0;

    virtual void setIsRunning(bool is_running) = 0;
    virtual void setJointTorqueOffset(const std::string& joint_name, double v) = 0;

    virtual void disableMotorDrive() = 0;
    virtual void clearFault() = 0;
    virtual void enableMotorDrive() = 0;
    virtual void switchToCSP() = 0;
    virtual void switchToCST() = 0;

    virtual void setIOState(const std::string& module_name, const std::string& interface_name, bool target_state) = 0;

    virtual void deleteToolFrame(const std::string& tool_name) = 0;
    virtual void addToolFrame(const std::string& tool_name, const KDL::Frame& frame) = 0;
    virtual void setCurrentToolFrame(const std::string& tool_name) = 0;
};

// Event subscription port for state fan-out.
class IRobotEvents {
public:
    virtual ~IRobotEvents() = default;

    virtual size_t registerMotorStatusCallback(MotorStatusCallback cb) = 0;
    virtual void unregisterMotorStatusCallback(size_t callback_id) = 0;

    virtual size_t registerIOStatusCallback(IOStatusCallback cb) = 0;
    virtual void unregisterIOStatusCallback(size_t callback_id) = 0;
};
