#include "motion_plugin.h"

#include <stdexcept>

MotionPlugin::MotionPlugin(
    const rclcpp::Node::SharedPtr& node,
    IRobotStateProvider* state_port,
    IRobotCommandPort* command_port,
    IKinematicsSolver* solver,
    IDynamicsService* dynamics)
    : node_(node), state_port_(state_port), command_port_(command_port),
      solver_(solver), dynamics_(dynamics)
{
    if (!node_ || !state_port_ || !command_port_ || !solver_ || !dynamics_) {
        throw std::invalid_argument("MotionPlugin requires non-null node, robot ports, solver, and dynamics service");
    }

    streaming_ = std::make_unique<StreamingController>(
        node_, state_port_, command_port_, solver_, dynamics_, state_machine_);
    trajectory_ = std::make_unique<TrajectoryController>(
        node_, state_port_, command_port_, solver_, state_machine_);
}

// ---------------------------------------------------------------------------
// Streaming motion
// ---------------------------------------------------------------------------

void MotionPlugin::jogCartesian(const std::array<double, 6>& velocity_arr) { streaming_->jogCartesian(velocity_arr); }
void MotionPlugin::jogCartesian(const geometry_msgs::msg::Twist& twist) { streaming_->jogCartesian(twist); }
void MotionPlugin::jogJoint(const std::string& joint_name, double direction, double velo_ratio) { streaming_->jogJoint(joint_name, direction, velo_ratio); }
void MotionPlugin::startDragging() { streaming_->startDragging(); }
void MotionPlugin::stopDragging() { streaming_->stopDragging(); }
bool MotionPlugin::isDragging() { return streaming_->isDragging(); }

// ---------------------------------------------------------------------------
// Discrete target motion
// ---------------------------------------------------------------------------

void MotionPlugin::moveToJointTarget(const JointsPosition& target, double velo_ratio)
{
    trajectory_->moveToJointTarget(target, velo_ratio);
}

void MotionPlugin::moveToJointTargetRelatively(const JointsPosition& delta, double velo_ratio)
{
    JointsPosition absolute;
    const auto& current = state_port_->getCurrentJointPosition();
    for (const auto& d : delta) {
        auto it = current.find(d.first);
        if (it == current.end()) continue;
        absolute[d.first].joint_value = d.second.joint_value + it->second.joint_value;
    }
    moveToJointTarget(absolute, velo_ratio);
}

void MotionPlugin::moveToPose(const KDL::Frame& pose, double velo_ratio)
{
    trajectory_->moveToPose(pose, velo_ratio);
}

bool MotionPlugin::isTrajectoryComplete()
{
    if (state_machine_.currentState() != MotionState::TrajectoryExec) return true;
    if (trajectory_->isComplete()) {
        state_machine_.forceIdle();
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// General
// ---------------------------------------------------------------------------

void MotionPlugin::stop()
{
    streaming_->stop();
    trajectory_->stop();
    state_machine_.forceIdle();
}

void MotionPlugin::stopStreaming() { streaming_->stopJog(); }

bool MotionPlugin::isRunning()
{
    return state_machine_.currentState() != MotionState::Idle;
}

// ---------------------------------------------------------------------------
// Kinematics delegates
// ---------------------------------------------------------------------------

geometry_msgs::msg::PoseStamped MotionPlugin::getCurrentCartesianPose() { return solver_->getCurrentCartesianPose(); }
void MotionPlugin::selectCoordinateSystem(const ControlCoordinateSystemType& coord_sys) { solver_->selectCoordinateSystem(coord_sys); }
void MotionPlugin::refreshCoordinateSystem() { solver_->refreshCoordinateSystem(); }
KDL::Frame MotionPlugin::tcpCalibration(const MovePointInfo& points) { return solver_->tcpCalibration(points); }

// ---------------------------------------------------------------------------
// Dynamics delegates
// ---------------------------------------------------------------------------

void MotionPlugin::identify(const size_t& db_start_index, const size_t& db_end_index) { dynamics_->identify(db_start_index, db_end_index); }
bool MotionPlugin::isDynamicsReady() const { return dynamics_->isReady(); }
const Eigen::MatrixXd& MotionPlugin::getDynamicsBaseParams() { return dynamics_->getBaseParams(); }
const Eigen::MatrixXd& MotionPlugin::getDynamicsFrictionParams() { return dynamics_->getFrictionParams(); }
const Eigen::MatrixXd& MotionPlugin::getDynamicsDepPb() { return dynamics_->getDepPb(); }
const Eigen::MatrixXd& MotionPlugin::getDynamicsDepPd() { return dynamics_->getDepPd(); }
const Eigen::MatrixXd& MotionPlugin::getDynamicsDepKd() { return dynamics_->getDepKd(); }

void MotionPlugin::setDynamicsParams(
    const Eigen::MatrixXd& base, const Eigen::MatrixXd& friction,
    const Eigen::MatrixXd& Pb, const Eigen::MatrixXd& Pd, const Eigen::MatrixXd& Kd)
{
    dynamics_->setParams(base, friction, Pb, Pd, Kd);
}
