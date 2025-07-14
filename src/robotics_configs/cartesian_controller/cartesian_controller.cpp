#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <Eigen/Dense>
#include "cartesian_controller.h"
#include "mode_changer.h"
#include "controller_switcher.h"
#include "move_executor.h"


CartesianController::CartesianController(const rclcpp::Node::SharedPtr& node)
    : m_Node(node) {

    m_EndEffector = "link5";
    m_FrameID = "link1";
    auto paramClient = std::make_shared<rclcpp::SyncParametersClient>(node, "controller_manager");

    while (!paramClient->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(node->get_logger(), "Waiting for controller_manager parameter service...");
    }

    if (paramClient->has_parameter("update_rate")) {
        m_UpdatePeriod = 1 / (double)paramClient->get_parameter<int>("update_rate");
        RCLCPP_INFO(node->get_logger(), "controller_manager update perioid: %f", m_UpdatePeriod);
    } else {
        RCLCPP_WARN(node->get_logger(), "controller_manager has no update_rate parameter");
    }

    paramClient = std::make_shared<rclcpp::SyncParametersClient>(node, "/robot_state_publisher");

    if (!paramClient->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node->get_logger(), "Parameter service for /robot_state_publisher not available.");
    }
    
    if (!paramClient->has_parameter("robot_description")) {
        RCLCPP_ERROR(node->get_logger(), "robot_description not found on /robot_state_publisher");
    }
    
    auto param = paramClient->get_parameter<std::string>(std::string("robot_description"));
    
    if (!m_URDFModel.initString(param)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to parse URDF string.");
    }
    if (!kdl_parser::treeFromUrdfModel(m_URDFModel, m_KDLTree)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to convert URDF to KDL tree.");
    }
    if (!m_KDLTree.getChain(m_FrameID, m_EndEffector, m_KDLChain)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get KDL chain.");
    }

    m_FKSolver = std::make_unique<KDL::ChainFkSolverPos_recursive>(m_KDLChain);
    m_JacSolver = std::make_shared<KDL::ChainJntToJacSolver>(m_KDLChain);
    double rate = m_Node->declare_parameter<double>("publishing_rate", 100.0);
    m_Timer = m_Node->create_wall_timer(std::chrono::duration<double>(1.0 / rate), std::bind(&CartesianController::publishPose, this));

    m_JointStateSubscriber = m_Node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,std::bind(&CartesianController::readPose, this, std::placeholders::_1));
    m_JointNums = m_KDLChain.segments.size();
    m_JointNames.resize(m_JointNums);
    m_JointPositions.resize(m_JointNums);
    m_JointLimitMin.resize(m_JointNums);
    m_JointLimitMax.resize(m_JointNums);
    for (size_t i = 0; i < m_JointNums; ++i) {
        if (m_KDLChain.segments[i].getJoint().getType() != KDL::Joint::None) {
            m_JointNames[i] = m_KDLChain.segments[i].getJoint().getName();
        }

        auto joint = m_URDFModel.getJoint(m_JointNames[i]);
        if (joint && joint->limits) {
            m_JointLimitMin(i) = joint->limits->lower;
            m_JointLimitMax(i) = joint->limits->upper;
        }
        else {
            m_JointLimitMin(i) = std::numeric_limits<double>::min();
            m_JointLimitMax(i) = std::numeric_limits<double>::max();
        }
    }
    m_IKSolver = std::make_unique<TRAC_IK::TRAC_IK>(m_Node, m_KDLChain, m_JointLimitMin, m_JointLimitMax);

    m_LastTime = m_Node->now();
}

void CartesianController::takeControl()
{
    m_TakingControl = true;
}

void CartesianController::dropControl()
{
    m_TakingControl = false;
}

void CartesianController::setVelocity(const std::array<double, 6> &velos)
{
    m_IsReady = true;
    m_Pose = m_PoseRT;
    for (size_t i = 0;i < velos.size(); ++i) {
        if(velos[i] != 0) {
            m_DriftSupression[i] = 0.0;
        }
        else {
            m_DriftSupression[i] = DRIFT_SUPRESSION_MULTIPLE;
        }
    }

    m_Velocity = velos;
}

std::array<double, 3> CartesianController::getPosition() const
{
    std::array<double, 3> res;
    res[0] = m_PoseRT.p.x();
    res[1] = m_PoseRT.p.y();
    res[2] = m_PoseRT.p.z();
    return res;
}

std::array<double, 3> CartesianController::getRotation() const
{
    std::array<double, 3> res;
    double z, y, x;
    m_PoseRT.M.GetEulerZYX(z, y, x);
    res[0] = x;
    res[1] = y;
    res[2] = z;
    return res;
}

void CartesianController::publishPose() {
    if(!m_IsReady) { return; }
    if(!m_TakingControl) { return; }
    sensor_msgs::msg::JointState msg;
    msg.velocity.resize(m_JointNums);
    msg.name.resize(m_JointNums);
    if(std::all_of(m_Velocity.begin(), m_Velocity.end(), [](const auto& v){return v == 0.0;})) {
        for (size_t i = 0; i < m_JointNums; ++i) {
            msg.velocity[i] = 0;
            msg.name[i] = m_JointNames[i];
        }
        MoveExecutor::instance().pubMoveCommand(msg);
        return;
    }
    Eigen::VectorXd cartesianVel(6);

    double Rx, Ry, Rz, RxRT, RyRT, RzRT;
    m_Pose.M.GetEulerZYX(Rz, Ry, Rx);
    m_PoseRT.M.GetEulerZYX(RzRT, RyRT, RxRT);

    cartesianVel <<
        m_Velocity[0] + (m_Pose.p.x() - m_PoseRT.p.x()) * m_DriftSupression[0],
        m_Velocity[1] + (m_Pose.p.y() - m_PoseRT.p.y()) * m_DriftSupression[1],
        m_Velocity[2] + (m_Pose.p.z() - m_PoseRT.p.z()) * m_DriftSupression[2],
        m_Velocity[3],
        m_Velocity[4],
        m_Velocity[5];

    KDL::Jacobian jacobian(m_JointNums);
    m_JacSolver->JntToJac(m_JointPositions, jacobian);

    Eigen::MatrixXd J(6, m_JointNums);
    for (int i = 0; i < 6; ++i)
        for (size_t j = 0; j < m_JointNums; ++j)
            J(i, j) = jacobian(i, j);


    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double sigmaMax = svd.singularValues()(0);
    double sigmaMin = svd.singularValues()(svd.singularValues().size() - 1);
    double cond = sigmaMax / sigmaMin;
    double lambda = MIN_LAMBDA;
    if (cond > m_LastCond) {
        if(cond > 1e3) {
            setVelocity({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            m_LastCond = cond;
            RCLCPP_WARN(m_Node->get_logger(), "Singularity detected, stopping motion.");
            return;
        }
        if (cond > LAMBDA_SLOPE) {
            lambda = MAX_LAMBDA;
        } else {
            lambda = MAX_LAMBDA - (LAMBDA_SLOPE - cond) / LAMBDA_SLOPE;
            if (lambda < MIN_LAMBDA) lambda = MIN_LAMBDA;
        }
    }
    Eigen::MatrixXd Jt = J.transpose();
    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd dq = Jt * ( (J * Jt + lambda * lambda * identity).inverse() ) * cartesianVel;

    for (size_t i = 0; i < m_JointNums; ++i) {
        msg.velocity[i] = dq(i);
        msg.name[i] = m_JointNames[i];
    }
    m_LastCond = cond;
    MoveExecutor::instance().pubMoveCommand(msg);
}

void CartesianController::readPose(const sensor_msgs::msg::JointState& msg)
{
    for (size_t i = 0; i < m_JointNames.size(); ++i) {
        auto it = std::find(msg.name.begin(), msg.name.end(), m_JointNames[i]);
        if (it != msg.name.end()) {
            size_t idx = std::distance(msg.name.begin(), it);
            if (idx < msg.position.size()) {
                m_JointPositions(i) = msg.position[idx];
            }
        }
    }
    m_FKSolver->JntToCart(m_JointPositions, m_PoseRT);
}

void CartesianController::scaleVelocity(double scaler)
{
    std::for_each(m_Velocity.begin(), m_Velocity.end(), [scaler](auto& v){v *= scaler;});
}
