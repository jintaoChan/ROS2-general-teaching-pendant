#include "trajectory_controller.h"
#include "trajectory_generator.h"
#include "motion_state_machine.h"

#include <stdexcept>
#include <magic_enum/magic_enum.hpp>

TrajectoryController::TrajectoryController(
    const rclcpp::Node::SharedPtr& node,
    IRobotStateProvider* state_port,
    IRobotCommandPort* command_port,
    IKinematicsSolver* solver,
    MotionStateMachine& state_machine)
    : node_(node), state_port_(state_port), command_port_(command_port),
      solver_(solver), state_machine_(state_machine)
{
}

void TrajectoryController::moveToJointTarget(const JointsPosition& target, double velo_ratio)
{
    if (!state_machine_.tryTransition(MotionState::TrajectoryExec)) {
        RCLCPP_WARN(node_->get_logger(), "Cannot start trajectory: motion state is %s",
            std::string(magic_enum::enum_name(state_machine_.currentState())).c_str());
        return;
    }

    const auto& joint_names = state_port_->getJointsName();
    const auto current = state_port_->getCurrentJointPosition();

    std::vector<double> start_pos, goal_pos, vel_limits;
    for (const auto& name : joint_names) {
        auto it_cur = current.find(name);
        if (it_cur == current.end()) continue;

        start_pos.push_back(it_cur->second.joint_value);
        auto it_tgt = target.find(name);
        goal_pos.push_back(it_tgt != target.end()
            ? it_tgt->second.joint_value
            : it_cur->second.joint_value);
        vel_limits.push_back(state_port_->getJointVelocityLimit(name));
    }

    auto traj = TrajectoryGenerator::generate(
        joint_names, start_pos, goal_pos, vel_limits, velo_ratio);
    command_port_->executeTrajectory(traj);
}

void TrajectoryController::moveToPose(const KDL::Frame& pose, double velo_ratio)
{
    auto target = solver_->solvePoseIK(pose);
    if (!target.has_value()) {
        throw std::runtime_error("Robot cannot reach this target pose");
    }
    moveToJointTarget(target.value(), velo_ratio);
}

bool TrajectoryController::isComplete() const
{
    return state_port_->isTrajectoryComplete();
}

void TrajectoryController::stop()
{
    auto current = state_port_->getCurrentJointPosition();
    command_port_->moveJointByAbsPosition(current, 0.0);
}
