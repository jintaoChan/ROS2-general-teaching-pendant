#pragma once

#include <memory>
#include <Eigen/Eigen>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "robot_ports.h"

namespace rclcpp {
class Node;
class Logger;
}

class IDynamicsService;

class DataBase;

class RobotHandle : public IRobotStateProvider,
                    public IRobotCommandPort,
                    public IRobotEvents {
public:
    explicit RobotHandle(const std::shared_ptr<rclcpp::Node>& node);
    ~RobotHandle() override;

    // IRobotStateProvider
    const urdf::Model& getURDFModel() const override;
    const KDL::Chain& getKDLChain() const override;
    JointsPosition getCurrentJointPosition() const override;
    JointsVelocity getCurrentJointVelocity() const override;
    JointsTorque getCurrentJointTorque() const override;
    size_t getJointNums() const override;
    double getJointVelocityLimit(const std::string&) const override;
    double getJointLowerLimit(const std::string&) const override;
    double getJointUpperLimit(const std::string&) const override;
    const std::vector<std::string>& getJointsName() const override;
    rclcpp::Time getTime() override;

    double getCartesianLimitsMaxTransVel() const override;
    double getCartesianLimitsMaxTransAcc() const override;
    double getCartesianLimitsMaxTransDec() const override;
    double getCartesianLimitsMaxRotVel() const override;

    uint64_t getControllerUpdatePeriod() const override;

    const std::string& getRobotArmBaseLinkName() const override;
    const std::string& getRobotArmEndLinkName() const override;
    const ToolInfo& getRobotArmToolInfo() const override;

    const bool& isToolFrameSet() const override;
    const std::string& getCurrentToolFrame() const override;

    bool isRunning() const override;
    bool isTrajectoryComplete() const override;

    const std::vector<std::string>& getIOInputGroupsName() const override;
    const std::vector<std::string>& getIOOutputGroupsName() const override;
    const std::vector<std::string>& getIOInterfacesName(const std::string& module_name) const override;
    bool isIOMonitorable(const std::string& module_name, const std::string& interface_name) const override;

    // IRobotCommandPort
    void moveJointByVelocity(const JointsVelocity& joint_velocity) override;
    void moveJointByAbsPosition(const JointsPosition& joint_position, double velo_ratio) override;
    void executeTrajectory(trajectory_msgs::msg::JointTrajectory& trajectory) override;
    void setJointTorque(const JointsTorque& joint_torque) override;

    void setIsRunning(bool is_running) override;

    void disableMotorDrive() override;
    void clearFault() override;
    void enableMotorDrive() override;
    void switchToCSP() override;
    void switchToCST() override;

    void setIOState(const std::string& module_name, const std::string& interface_name, bool target_state) override;
    void deleteToolFrame(const std::string& tool_name) override;
    void addToolFrame(const std::string& tool_name, const KDL::Frame& frame) override;
    void setCurrentToolFrame(const std::string& tool_name) override;

    // IRobotEvents
    size_t registerMotorStatusCallback(MotorStatusCallback cb) override;
    void unregisterMotorStatusCallback(size_t callback_id) override;
    size_t registerIOStatusCallback(IOStatusCallback cb) override;
    void unregisterIOStatusCallback(size_t callback_id) override;

    // RobotHandle-specific (not in interfaces)
    void setDynamicsService(const std::shared_ptr<IDynamicsService>& dynamics);


private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

