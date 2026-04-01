#pragma once

#include <rclcpp/rclcpp.hpp>
#include <kdl/frames.hpp>

#include "robot_ports.h"
#include "i_kinematics_solver.h"

class MotionStateMachine;

class TrajectoryController {
public:
    TrajectoryController(
        const rclcpp::Node::SharedPtr& node,
        IRobotStateProvider* state_port,
        IRobotCommandPort* command_port,
        IKinematicsSolver* solver,
        MotionStateMachine& state_machine);

    void moveToJointTarget(const JointsPosition& target, double velo_ratio);
    void moveToPose(const KDL::Frame& pose, double velo_ratio);
    bool isComplete() const;
    void stop();

private:
    rclcpp::Node::SharedPtr node_;
    IRobotStateProvider* state_port_;
    IRobotCommandPort* command_port_;
    IKinematicsSolver* solver_;
    MotionStateMachine& state_machine_;
};
