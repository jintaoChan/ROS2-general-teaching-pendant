#include "robot_handle_ports_adapter.h"

const urdf::Model& RobotHandlePortsAdapter::getURDFModel() const { return robot_handle_.getURDFModel(); }
const KDL::Chain& RobotHandlePortsAdapter::getKDLChain() const { return robot_handle_.getKDLChain(); }

size_t RobotHandlePortsAdapter::getJointNums() const { return robot_handle_.getJointNums(); }
const std::vector<std::string>& RobotHandlePortsAdapter::getJointsName() const { return robot_handle_.getJointsName(); }

const JointsPosition& RobotHandlePortsAdapter::getCurrentJointPosition() const { return robot_handle_.getCurrentJointPosition(); }
const JointsVelocity& RobotHandlePortsAdapter::getCurrentJointVelocity() const { return robot_handle_.getCurrentJointVelocity(); }
const JointsTorque& RobotHandlePortsAdapter::getCurrentJointTorque() const { return robot_handle_.getCurrentJointTorque(); }
const JointsTorque& RobotHandlePortsAdapter::getCurrentJointEstimatedExternalTorque() const { return robot_handle_.getCurrentJointEstimatedExternalTorque(); }

const double& RobotHandlePortsAdapter::getJointVelocityLimit(const std::string& j_name) const { return robot_handle_.getJointVelocityLimit(j_name); }
const double& RobotHandlePortsAdapter::getJointLowerLimit(const std::string& j_name) const { return robot_handle_.getJointLowerLimit(j_name); }
const double& RobotHandlePortsAdapter::getJointUpperLimit(const std::string& j_name) const { return robot_handle_.getJointUpperLimit(j_name); }

const std::string& RobotHandlePortsAdapter::getRobotArmBaseLinkName() const { return robot_handle_.getRobotArmBaseLinkName(); }
const std::string& RobotHandlePortsAdapter::getRobotArmEndLinkName() const { return robot_handle_.getRobotArmEndLinkName(); }
const ToolInfo& RobotHandlePortsAdapter::getRobotArmToolInfo() const { return robot_handle_.getRobotArmToolInfo(); }
const DragParams& RobotHandlePortsAdapter::getDragParams() const { return robot_handle_.getDragParams(); }

const bool& RobotHandlePortsAdapter::isToolFrameSet() const { return robot_handle_.isToolFrameSet(); }
const std::string& RobotHandlePortsAdapter::getCurrentToolFrame() const { return robot_handle_.getCurrentToolFrame(); }

const double& RobotHandlePortsAdapter::getCartesianLimitsMaxTransVel() const { return robot_handle_.getCartesianLimitsMaxTransVel(); }
const double& RobotHandlePortsAdapter::getCartesianLimitsMaxTransAcc() const { return robot_handle_.getCartesianLimitsMaxTransAcc(); }
const double& RobotHandlePortsAdapter::getCartesianLimitsMaxTransDec() const { return robot_handle_.getCartesianLimitsMaxTransDec(); }
const double& RobotHandlePortsAdapter::getCartesianLimitsMaxRotVel() const { return robot_handle_.getCartesianLimitsMaxRotVel(); }

const uint64_t& RobotHandlePortsAdapter::getControllerUpdatePeriod() const { return robot_handle_.getControllerUpdatePeriod(); }
rclcpp::Time RobotHandlePortsAdapter::getTime() { return robot_handle_.getTime(); }

const bool& RobotHandlePortsAdapter::isRunning() const { return robot_handle_.isRunning(); }

const std::vector<std::string>& RobotHandlePortsAdapter::getIOInputGroupsName() const { return robot_handle_.getIOInputGroupsName(); }
const std::vector<std::string>& RobotHandlePortsAdapter::getIOOutputGroupsName() const { return robot_handle_.getIOOutputGroupsName(); }
const std::vector<std::string>& RobotHandlePortsAdapter::getIOInterfacesName(const std::string& module_name) const { return robot_handle_.getIOInterfacesName(module_name); }
bool RobotHandlePortsAdapter::isIOMonitorable(const std::string& module_name, const std::string& interface_name) const { return robot_handle_.isIOMonitorable(module_name, interface_name); }

void RobotHandlePortsAdapter::moveJointByVelcoity(const JointsVelocity& joint_velocity) { robot_handle_.moveJointByVelcoity(joint_velocity); }
void RobotHandlePortsAdapter::moveJointByAbsPosition(const JointsPosition& joint_position, double velo_ratio) { robot_handle_.moveJointByAbsPosition(joint_position, velo_ratio); }
void RobotHandlePortsAdapter::moveJointByAbsPosition(trajectory_msgs::msg::JointTrajectory& joint_position) { robot_handle_.moveJointByAbsPosition(joint_position); }
void RobotHandlePortsAdapter::setJointTorque(const JointsTorque& joint_torque) { robot_handle_.setJointTorque(joint_torque); }

void RobotHandlePortsAdapter::setIsRunning(bool is_running) { robot_handle_.setIsRunning(is_running); }
void RobotHandlePortsAdapter::setJointTorqueOffset(const std::string& joint_name, double v) { robot_handle_.setJointTorqueOffset(joint_name, v); }

void RobotHandlePortsAdapter::disableMotorDrive() { robot_handle_.disableMotorDrive(); }
void RobotHandlePortsAdapter::clearFault() { robot_handle_.clearFault(); }
void RobotHandlePortsAdapter::enableMotorDrive() { robot_handle_.enableMotorDrive(); }
void RobotHandlePortsAdapter::switchToCSP() { robot_handle_.switchToCSP(); }
void RobotHandlePortsAdapter::switchToCST() { robot_handle_.switchToCST(); }

void RobotHandlePortsAdapter::setIOState(const std::string& module_name, const std::string& interface_name, bool target_state) {
    robot_handle_.setIOState(module_name, interface_name, target_state);
}

void RobotHandlePortsAdapter::deleteToolFrame(const std::string& tool_name) {
    robot_handle_.deleteToolFrame(tool_name);
}

void RobotHandlePortsAdapter::addToolFrame(const std::string& tool_name, const KDL::Frame& frame) {
    robot_handle_.addToolFrame(tool_name, frame);
}

void RobotHandlePortsAdapter::setCurrentToolFrame(const std::string& tool_name) {
    robot_handle_.setCurrentToolFrame(tool_name);
}

size_t RobotHandlePortsAdapter::registerMotorStatusCallback(MotorStatusCallback cb) {
    return robot_handle_.registerMotorStatusCallback(std::move(cb));
}

void RobotHandlePortsAdapter::unregisterMotorStatusCallback(size_t callback_id) {
    robot_handle_.unregisterMotorStatusCallback(callback_id);
}

size_t RobotHandlePortsAdapter::registerIOStatusCallback(IOStatusCallback cb) {
    return robot_handle_.registerIOStatusCallback(std::move(cb));
}

void RobotHandlePortsAdapter::unregisterIOStatusCallback(size_t callback_id) {
    robot_handle_.unregisterIOStatusCallback(callback_id);
}
