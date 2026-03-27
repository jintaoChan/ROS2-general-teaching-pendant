#pragma once

#include "robot_ports.h"

// Adapter that exposes RobotHandle through decoupled ports.
class RobotHandlePortsAdapter : public IRobotStateProvider,
                                public IRobotCommandPort,
                                public IRobotEvents {
public:
    explicit RobotHandlePortsAdapter(RobotHandle& robot_handle)
        : robot_handle_(robot_handle) {}

    const urdf::Model& getURDFModel() const override;
    const KDL::Chain& getKDLChain() const override;

    size_t getJointNums() const override;
    const std::vector<std::string>& getJointsName() const override;

    const JointsPosition& getCurrentJointPosition() const override;
    const JointsVelocity& getCurrentJointVelocity() const override;
    const JointsTorque& getCurrentJointTorque() const override;
    const JointsTorque& getCurrentJointEstimatedExternalTorque() const override;

    const double& getJointVelocityLimit(const std::string& j_name) const override;
    const double& getJointLowerLimit(const std::string& j_name) const override;
    const double& getJointUpperLimit(const std::string& j_name) const override;

    const std::string& getRobotArmBaseLinkName() const override;
    const std::string& getRobotArmEndLinkName() const override;
    const ToolInfo& getRobotArmToolInfo() const override;
    const DragParams& getDragParams() const override;

    const bool& isToolFrameSet() const override;
    const std::string& getCurrentToolFrame() const override;

    const double& getCartesianLimitsMaxTransVel() const override;
    const double& getCartesianLimitsMaxTransAcc() const override;
    const double& getCartesianLimitsMaxTransDec() const override;
    const double& getCartesianLimitsMaxRotVel() const override;

    const uint64_t& getControllerUpdatePeriod() const override;
    rclcpp::Time getTime() override;

    const bool& isRunning() const override;

    const std::vector<std::string>& getIOInputGroupsName() const override;
    const std::vector<std::string>& getIOOutputGroupsName() const override;
    const std::vector<std::string>& getIOInterfacesName(const std::string& module_name) const override;
    bool isIOMonitorable(const std::string& module_name, const std::string& interface_name) const override;

    void moveJointByVelcoity(const JointsVelocity& joint_velocity) override;
    void moveJointByAbsPosition(const JointsPosition& joint_position, double velo_ratio) override;
    void moveJointByAbsPosition(trajectory_msgs::msg::JointTrajectory& joint_position) override;
    void setJointTorque(const JointsTorque& joint_torque) override;

    void setIsRunning(bool is_running) override;
    void setJointTorqueOffset(const std::string& joint_name, double v) override;

    void disableMotorDrive() override;
    void clearFault() override;
    void enableMotorDrive() override;
    void switchToCSP() override;
    void switchToCST() override;

    void setIOState(const std::string& module_name, const std::string& interface_name, bool target_state) override;
    void deleteToolFrame(const std::string& tool_name) override;
    void addToolFrame(const std::string& tool_name, const KDL::Frame& frame) override;
    void setCurrentToolFrame(const std::string& tool_name) override;

    size_t registerMotorStatusCallback(MotorStatusCallback cb) override;
    void unregisterMotorStatusCallback(size_t callback_id) override;

    size_t registerIOStatusCallback(IOStatusCallback cb) override;
    void unregisterIOStatusCallback(size_t callback_id) override;

private:
    RobotHandle& robot_handle_;
};
