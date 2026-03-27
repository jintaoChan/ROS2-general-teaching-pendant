#pragma once

#include <atomic>
#include <mutex>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <kdl/frames.hpp>
#include <Eigen/Dense>

#include "singleton.hpp"
#include "i_kinematics_solver.h"
#include "i_dynamics_service.h"
#include "robot_ports.h"

// MotionPlugin is the motion-execution entry point for upper-layer business logic.
// Current implementation delegates to KinematicsPlugin to preserve behavior while
// we incrementally migrate execution logic out of kinematics.
class MotionPlugin : public Singleton<MotionPlugin> {
    friend class Singleton<MotionPlugin>;

public:
    MotionPlugin(
        const rclcpp::Node::SharedPtr& node,
        IRobotStateProvider* state_port,
        IRobotCommandPort* command_port,
        IKinematicsSolver* solver,
        IDynamicsService* dynamics);
    ~MotionPlugin() = default;

public:
    void twistRobot(const std::array<double, 6>& velocity_arr);
    void twistRobot(const geometry_msgs::msg::Twist& twist_msg);
    void moveJointPositionRelatively(const JointsPosition& pos, double velo_ratio);
    void moveJointPositionAbsolutely(const JointsPosition& pos, double velo_ratio);
    void moveToPose(const KDL::Frame& pose, double velo_ratio);
    void stop();
    void startDragging();
    void stopDragging();
    bool isDragging();
    bool isRunning();

    geometry_msgs::msg::PoseStamped getCurrentCartesianPose();
    void selectCoordinateSystem(const ControlCoordinateSystemType& coord_sys);
    void refreshCoordinateSystem();
    KDL::Frame tcpCalibration(const MovePointInfo& points);

    // Dynamics / parameter identification delegates
    void identify(const size_t& db_start_index, const size_t& db_end_index);
    bool isDynamicsReady() const;
    const Eigen::MatrixXd& getDynamicsBaseParams();
    const Eigen::MatrixXd& getDynamicsFrictionParams();
    const Eigen::MatrixXd& getDynamicsDepPb();
    const Eigen::MatrixXd& getDynamicsDepPd();
    const Eigen::MatrixXd& getDynamicsDepKd();
    void setDynamicsParams(const Eigen::MatrixXd& base, const Eigen::MatrixXd& friction,
                           const Eigen::MatrixXd& Pb, const Eigen::MatrixXd& Pd, const Eigen::MatrixXd& Kd);

private:
    void cartesianJogCallback();
    void processCartesianJog();
    void jointJogCallback();
    void processJointJog();
    void dragCallback();
    void processDrag();

private:
    rclcpp::Node::SharedPtr node_;
    IRobotStateProvider* state_port_{nullptr};
    IRobotCommandPort* command_port_{nullptr};
    double velo_ratio_{0.0};
    std::mutex cartesian_jog_mutex_;
    std::atomic<bool> has_pending_cartesian_jog_task_{false};
    rclcpp::TimerBase::SharedPtr cartesian_jog_timer_;

    sensor_msgs::msg::JointState stored_joint_position_;
    sensor_msgs::msg::JointState target_joint_position_;
    std::mutex joint_jog_mutex_;
    std::atomic<bool> has_pending_joint_jog_task_{false};
    rclcpp::TimerBase::SharedPtr joint_jog_timer_;

    std::mutex drag_mutex_;
    std::atomic<bool> has_pending_drag_task_{false};
    rclcpp::TimerBase::SharedPtr drag_timer_;
    IKinematicsSolver* solver_{nullptr};
    IDynamicsService* dynamics_{nullptr};
};
