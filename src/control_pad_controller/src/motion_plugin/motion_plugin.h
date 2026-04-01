#pragma once

#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <kdl/frames.hpp>
#include <Eigen/Dense>

#include "singleton.hpp"
#include "i_kinematics_solver.h"
#include "i_dynamics_service.h"
#include "robot_ports.h"
#include "motion_state_machine.h"
#include "streaming_controller.h"
#include "trajectory_controller.h"

// MotionPlugin is the motion-execution entry point for upper-layer business logic.
// Internally delegates to StreamingController (jog/drag) and TrajectoryController
// (discrete target motion), with a MotionStateMachine enforcing mutual exclusion.
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
    // Streaming motion (jog / drag)
    void jogCartesian(const std::array<double, 6>& velocity_arr);
    void jogCartesian(const geometry_msgs::msg::Twist& twist);
    void jogJoint(const std::string& joint_name, double direction, double velo_ratio);
    void startDragging();
    void stopDragging();
    bool isDragging();

    // Discrete target motion (trajectory)
    void moveToJointTarget(const JointsPosition& target, double velo_ratio);
    void moveToJointTargetRelatively(const JointsPosition& delta, double velo_ratio);
    void moveToPose(const KDL::Frame& pose, double velo_ratio);
    bool isTrajectoryComplete();

    // General
    void stop();
    void stopStreaming();
    bool isRunning();

    // Kinematics delegates
    geometry_msgs::msg::PoseStamped getCurrentCartesianPose();
    void selectCoordinateSystem(const ControlCoordinateSystemType& coord_sys);
    void refreshCoordinateSystem();
    KDL::Frame tcpCalibration(const MovePointInfo& points);

    // Dynamics delegates
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
    rclcpp::Node::SharedPtr node_;
    IRobotStateProvider* state_port_{nullptr};
    IRobotCommandPort* command_port_{nullptr};
    IKinematicsSolver* solver_{nullptr};
    IDynamicsService* dynamics_{nullptr};

    MotionStateMachine state_machine_;
    std::unique_ptr<StreamingController> streaming_;
    std::unique_ptr<TrajectoryController> trajectory_;
};
