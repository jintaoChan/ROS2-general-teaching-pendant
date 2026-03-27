#pragma once
#include <memory>
#include <geometry_msgs/msg/pose.hpp>
#include <urdf/model.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <trac_ik/trac_ik.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include "i_kinematics_solver.h"
#include "robot_ports.h"

class KinematicsPlugin : public IKinematicsSolver {
public:
    explicit KinematicsPlugin(IRobotStateProvider* state_port);
    ~KinematicsPlugin() = default;

public:
    void prepareCartesianJog(const geometry_msgs::msg::Twist& twist_msg) override;
    std::optional<JointsPosition> computeCartesianJogCommand(const rclcpp::Time& current_time) override;
    std::optional<JointsPosition> solvePoseIK(const KDL::Frame& pose) override;
    geometry_msgs::msg::PoseStamped getCurrentCartesianPose() override;
    void selectCoordinateSystem(const ControlCoordinateSystemType& coord_sys) override;
    void refreshCoordinateSystem() override;
    KDL::Frame tcpCalibration(const MovePointInfo& points) override;

private:
    void resetSolver();
    KDL::Frame tcpCalibration(const std::vector<KDL::Frame>& points);

private:
    KDL::Frame pose2KDLFrame(geometry_msgs::msg::Pose pose);
    geometry_msgs::msg::Pose kdlFrame2Pose(KDL::Frame frame);

private:
    IRobotStateProvider* state_port_{nullptr};
    urdf::Model urdf_model_;
    std::string root_frame_id_;
    ControlCoordinateSystemType coord_sys_type_{ControlCoordinateSystemType::Base};
    KDL::Chain default_kdl_chain_;
    KDL::Chain kdl_chain_;
    KDL::Segment tool_segment_;
    size_t joints_num_;
    std::vector<std::string> joints_names_;
    KDL::JntArray joints_limit_min_;
    KDL::JntArray joints_limit_max_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_velocity_solver_;
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_;
    KDL::JntArray current_joint_position_;
    KDL::Rotation selected_coord_system_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped stored_pose_;
    geometry_msgs::msg::Twist twist_msg_;
};
