#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "i_kinematics_solver.h"
#include "i_dynamics_service.h"
#include "robot_ports.h"

class MotionStateMachine;

class StreamingController {
public:
    StreamingController(
        const rclcpp::Node::SharedPtr& node,
        IRobotStateProvider* state_port,
        IRobotCommandPort* command_port,
        IKinematicsSolver* solver,
        IDynamicsService* dynamics,
        MotionStateMachine& state_machine);

    void jogCartesian(const geometry_msgs::msg::Twist& twist);
    void jogCartesian(const std::array<double, 6>& velocity_arr);
    void jogJoint(const std::string& joint_name, double direction, double velo_ratio);

    void startDragging();
    void stopDragging();
    bool isDragging() const;

    void stopJog();
    void stop();

private:
    void cartesianJogCallback();
    void processCartesianJog();
    void jointJogCallback();
    void processJointJog();
    void dragCallback();
    void processDrag();

    rclcpp::Node::SharedPtr node_;
    IRobotStateProvider* state_port_;
    IRobotCommandPort* command_port_;
    IKinematicsSolver* solver_;
    IDynamicsService* dynamics_;
    MotionStateMachine& state_machine_;

    std::mutex cartesian_jog_mutex_;
    std::atomic<bool> has_pending_cartesian_jog_task_{false};
    rclcpp::TimerBase::SharedPtr cartesian_jog_timer_;

    std::string jog_joint_name_;
    double jog_direction_{0.0};
    double jog_velo_ratio_{0.0};
    double jog_commanded_pos_{0.0};   // open-loop commanded position (avoids feedback lag)
    std::mutex joint_jog_mutex_;
    std::atomic<bool> has_pending_joint_jog_task_{false};
    rclcpp::TimerBase::SharedPtr joint_jog_timer_;

    std::mutex drag_mutex_;
    std::atomic<bool> has_pending_drag_task_{false};
    rclcpp::TimerBase::SharedPtr drag_timer_;
};
