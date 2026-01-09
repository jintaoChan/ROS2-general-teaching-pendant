#pragma once
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <urdf/model.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <trac_ik/trac_ik.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include "singleton.hpp"
#include "robot_handle.h"

class KinematicsPlugin : public Singleton<KinematicsPlugin>{
    friend class Singleton<KinematicsPlugin>;
public:
    KinematicsPlugin(const rclcpp::Node::SharedPtr& node);
    ~KinematicsPlugin();

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
    void refreshCoordinateSystem();// tool interchange

    KDL::Frame tcpCalibration(const MovePointInfo& points);

private:
    void cartesianJogCallback();
    void jointJogCallback();
    void dragCallback();
    void processCartesianJog();
    void processJointJog();
    void processDrag();

    void remoteCartesianJogCallback(const geometry_msgs::msg::TwistStamped& msg);
    void remoteJointJogCallback(const control_msgs::msg::JointJog& msg);
    void remoteCartesianMoveCallback(const geometry_msgs::msg::PoseStamped& msg);

private:
    void resetSolver();
    KDL::Frame tcpCalibration(const std::vector<KDL::Frame>& points);

private:
    KDL::Frame pose2KDLFrame(geometry_msgs::msg::Pose pose);
    geometry_msgs::msg::Pose kdlFrame2Pose(KDL::Frame frame);

private:
    rclcpp::Node::SharedPtr node_;
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
    std::mutex worker_mutex_;
    std::atomic<bool> has_pending_cartesian_jog_task_{false};
    std::atomic<bool> has_pending_joint_jog_task_{false};
    std::atomic<bool> has_pending_drag_task_{false};
    std::thread worker_thread_;
    bool stop_worker_{false};
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    double velo_ratio_{0};
    KDL::JntArray current_joint_position_;
    KDL::Rotation selected_coord_system_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped stored_pose_;
    sensor_msgs::msg::JointState stored_joint_position_;
    rclcpp::TimerBase::SharedPtr cartesian_jog_timer_;
    rclcpp::TimerBase::SharedPtr joint_jog_timer_;
    rclcpp::TimerBase::SharedPtr drag_timer_;
    geometry_msgs::msg::Twist twist_msg_;
    sensor_msgs::msg::JointState target_joint_position_;
};
