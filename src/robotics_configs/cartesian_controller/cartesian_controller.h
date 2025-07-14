#ifndef CARTESIAN_CONTROLLER_H
#define CARTESIAN_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "singleton.hpp"


using namespace std::chrono_literals;

class CartesianController : public Singleton<CartesianController> {
    friend class Singleton<CartesianController>;
public:
    CartesianController(const rclcpp::Node::SharedPtr& node);

public:
    void setVelocity(const std::array<double, 6>& velos);
    void takeControl();
    void dropControl();
    void clearVelocity();
    std::array<double, 3> getPosition() const;
    std::array<double, 3> getRotation() const;

private:
    void publishPose();
    void readPose(const sensor_msgs::msg::JointState& msg);
    void scaleVelocity(double scaler);

private:
    rclcpp::Node::SharedPtr m_Node;
    std::string m_PoseTopic{"cartesian_motion_controller/target_frame"}, m_FrameID, m_EndEffector;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_Publisher;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_JointStateSubscriber;
    rclcpp::TimerBase::SharedPtr m_Timer;
    urdf::Model m_URDFModel;
    KDL::Tree m_KDLTree;
    KDL::Chain m_KDLChain;
    size_t m_JointNums;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> m_FKSolver;
    std::unique_ptr<TRAC_IK::TRAC_IK> m_IKSolver;
    std::vector<std::string> m_JointNames;
    KDL::JntArray m_JointPositions;
    KDL::JntArray m_JointLimitMin;
    KDL::JntArray m_JointLimitMax;
    KDL::Frame m_Pose;
    KDL::Frame m_PoseRT;
    rclcpp::Time m_LastTime;
    std::shared_ptr<KDL::ChainJntToJacSolver> m_JacSolver;
    std::array<double, 6> m_Velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 6> m_DriftSupression = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double DRIFT_SUPRESSION_MULTIPLE = 100.0;
    double m_UpdatePeriod;
    bool m_IsReady{false};
    bool m_TakingControl{false};
    double m_LastCond = std::numeric_limits<double>::min();
    const double LAMBDA_SLOPE = 100;
    const double MIN_LAMBDA = 0.01;
    const double MAX_LAMBDA = 0.1;
};

#endif // CARTESIAN_CONTROLLER_H
