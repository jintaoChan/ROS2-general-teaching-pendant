#include "kinematics_plugin.h"
#include <Eigen/Dense>
#include <iostream>
#include <stdexcept>

KinematicsPlugin::KinematicsPlugin(IRobotStateProvider* state_port)
    : state_port_(state_port)
{
    if (state_port_ == nullptr) {
        throw std::invalid_argument("KinematicsPlugin requires non-null state port");
    }

    urdf_model_ = state_port_->getURDFModel();
    default_kdl_chain_ = state_port_->getKDLChain();
    tool_segment_ = KDL::Segment("tool_link", KDL::Joint("tool_joint"));
    joints_num_ = default_kdl_chain_.getNrOfJoints();
    current_joint_position_.resize(joints_num_);

    joints_limit_min_.resize(joints_num_);
    joints_limit_max_.resize(joints_num_);
    for (size_t i = 0, idx = 0; i < default_kdl_chain_.segments.size(); ++i) {
        if(default_kdl_chain_.segments[i].getJoint().getType() == KDL::Joint::None) {continue;}
        const auto& joint_name = default_kdl_chain_.segments[i].getJoint().getName();
        joints_names_.push_back(joint_name);
        auto joint = urdf_model_.getJoint(joint_name);
        if (joint && joint->limits) {
            joints_limit_min_(idx) = joint->limits->lower;
            joints_limit_max_(idx) = joint->limits->upper;
        }
        else {
            joints_limit_min_(idx) = -std::numeric_limits<double>::max();
            joints_limit_max_(idx) = std::numeric_limits<double>::max();
        }
        ++idx;
    }
    resetSolver();
    current_pose_.header.frame_id = root_frame_id_;
}

void KinematicsPlugin::prepareCartesianJog(const geometry_msgs::msg::Twist& twist_msg)
{
    twist_msg_ = twist_msg;
    stored_pose_ = current_pose_;
    switch(coord_sys_type_) {
    case ControlCoordinateSystemType::Tool: {
        selected_coord_system_ = KDL::Rotation::Quaternion(stored_pose_.pose.orientation.x, stored_pose_.pose.orientation.y, stored_pose_.pose.orientation.z, stored_pose_.pose.orientation.w);
        break;
    }
    case ControlCoordinateSystemType::Base: {
        selected_coord_system_ = KDL::Rotation::Identity();
        break;
    }
    case ControlCoordinateSystemType::EndEffector: {
        selected_coord_system_ = KDL::Rotation::Quaternion(stored_pose_.pose.orientation.x, stored_pose_.pose.orientation.y, stored_pose_.pose.orientation.z, stored_pose_.pose.orientation.w);
        break;
    }
    }
    resetSolver();
}

std::optional<JointsPosition> KinematicsPlugin::computeCartesianJogCommand(const rclcpp::Time& current_time)
{
    const auto joints_value_map = state_port_->getCurrentJointPosition();
    for (size_t i = 0; i < joints_num_; ++i) {
        current_joint_position_(i) = joints_value_map.at(joints_names_[i]).joint_value;
    }

    auto time_diff = current_time.seconds() -
        (static_cast<double>(stored_pose_.header.stamp.sec) + 1e-9 * static_cast<double>(stored_pose_.header.stamp.nanosec));

    bool is_ok{false};
    static const double max_velocity_divider{4};
    double velocity_divider{1};
    KDL::JntArray target_joints_value(joints_num_);
    KDL::Frame target_pose_start = pose2KDLFrame(stored_pose_.pose);

    while (!is_ok && velocity_divider < max_velocity_divider) {
        KDL::Frame target_pose = target_pose_start;
        auto transformed_linear_vel = selected_coord_system_ * KDL::Vector(twist_msg_.linear.x, twist_msg_.linear.y, twist_msg_.linear.z);
        auto vx = transformed_linear_vel(0);
        auto vy = transformed_linear_vel(1);
        auto vz = transformed_linear_vel(2);

        target_pose.p.x(target_pose.p.x() + time_diff * vx / velocity_divider);
        target_pose.p.y(target_pose.p.y() + time_diff * vy / velocity_divider);
        target_pose.p.z(target_pose.p.z() + time_diff * vz / velocity_divider);
        target_pose.M = selected_coord_system_ *
                        KDL::Rotation::RotX(time_diff * twist_msg_.angular.x / velocity_divider) *
                        KDL::Rotation::RotY(time_diff * twist_msg_.angular.y / velocity_divider) *
                        KDL::Rotation::RotZ(time_diff * twist_msg_.angular.z / velocity_divider) *
                        selected_coord_system_.Inverse() * target_pose.M;

        auto ik_ret = ik_solver_->CartToJnt(current_joint_position_, target_pose, target_joints_value);
        if (ik_ret < 0) {
            return std::nullopt;
        }

        bool is_velocity_valid{true};
        for (size_t i = 0; i < joints_num_; ++i) {
            if (std::abs(target_joints_value(i) - current_joint_position_(i)) > state_port_->getJointVelocityLimit(joints_names_[i]) * time_diff) {
                is_velocity_valid = false;
                break;
            }
        }
        if (!is_velocity_valid) {
            velocity_divider *= 2;
            continue;
        }
        is_ok = true;
    }

    if (!is_ok) {
        return std::nullopt;
    }

    JointsPosition target_joint_positions;
    for (size_t i = 0; i < joints_num_; ++i) {
        target_joint_positions[joints_names_[i]].joint_value = target_joints_value(i);
    }
    return target_joint_positions;
}

std::optional<JointsPosition> KinematicsPlugin::solvePoseIK(const KDL::Frame& pose)
{
    const auto joints_value_map = state_port_->getCurrentJointPosition();
    for (size_t i = 0; i < joints_num_; ++i) {
        current_joint_position_(i) = joints_value_map.at(joints_names_[i]).joint_value;
    }

    KDL::JntArray target_joints_value(joints_num_);
    auto ik_ret = ik_solver_->CartToJnt(current_joint_position_, pose, target_joints_value);
    if (ik_ret < 0) {
        return std::nullopt;
    }

    JointsPosition target_joint_positions;
    for (size_t i = 0; i < joints_num_; ++i) {
        target_joint_positions[joints_names_[i]].joint_value = target_joints_value(i);
    }
    return target_joint_positions;
}

geometry_msgs::msg::PoseStamped KinematicsPlugin::getCurrentCartesianPose()
{
    KDL::Frame pose;
    auto joints_value_map = state_port_->getCurrentJointPosition();
    for (size_t i = 0;i < joints_num_; ++i) {
        current_joint_position_(i) = joints_value_map[joints_names_[i]].joint_value;
    }
    fk_solver_->JntToCart(current_joint_position_, pose);

    current_pose_.pose.position.x  = pose.p.x();
    current_pose_.pose.position.y  = pose.p.y();
    current_pose_.pose.position.z  = pose.p.z();
    pose.M.GetQuaternion(current_pose_.pose.orientation.x, current_pose_.pose.orientation.y, current_pose_.pose.orientation.z, current_pose_.pose.orientation.w);
    current_pose_.header.stamp = state_port_->getTime();
    return current_pose_;
}

void KinematicsPlugin::selectCoordinateSystem(const ControlCoordinateSystemType& coord_sys)
{
    coord_sys_type_ = coord_sys;
    switch(coord_sys_type_) {
    case ControlCoordinateSystemType::Tool:{
        auto current_tool_frame = state_port_->getCurrentToolFrame();
        tool_segment_.setFrameToTip(state_port_->getRobotArmToolInfo().at(current_tool_frame));
        break;
    }
    default:
        tool_segment_.setFrameToTip(KDL::Frame::Identity());
    }
    resetSolver();
}

void KinematicsPlugin::refreshCoordinateSystem() {
    selectCoordinateSystem(coord_sys_type_);
}

KDL::Frame KinematicsPlugin::tcpCalibration(const MovePointInfo &points)
{
    std::vector<KDL::Frame> input;
    input.reserve(points.size());
    auto fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(default_kdl_chain_);

    for(const auto& p : points) {
        KDL::JntArray for_fk(joints_num_);
        KDL::Frame frame;
        const auto& joint_infos = p.second.JointValues;
        for(const auto& j : joint_infos) {
            auto name_it = std::find(joints_names_.begin(), joints_names_.end(), j.first);
            if(name_it != joints_names_.end()) {
                auto name_idx = std::distance(joints_names_.begin(), name_it);
                for_fk(name_idx) = j.second.joint_value;
            }
        }
        fk_solver->JntToCart(for_fk, frame);
        input.push_back(frame);
    }
    return tcpCalibration(input);
}

KDL::Frame KinematicsPlugin::tcpCalibration(const std::vector<KDL::Frame> &points)
{
    const auto& size = points.size();
    if(size < 3) {
        std::cerr << "[KinematicsPlugin::tcpCalibration] Need at least 3 poses" << std::endl;
        return {};
    }
    int row_size = 3 * (int)std::floor(size * (size - 1) / 2);
    Eigen::MatrixXd A(row_size, 3), b(row_size, 1);
    size_t count = 0;
    for(size_t i = 0; i < size - 1; ++i) {
        const auto& r1 = points[i].M;
        const auto& t1 = points[i].p;
        for(size_t j = i + 1; j < size; ++j) {
            const auto& r2 = points[j].M;
            const auto& t2 = points[j].p;
            A.row(3 * count)     << r1.data[0] - r2.data[0], r1.data[1] - r2.data[1], r1.data[2] - r2.data[2];
            A.row(3 * count + 1) << r1.data[3] - r2.data[3], r1.data[4] - r2.data[4], r1.data[5] - r2.data[5];
            A.row(3 * count + 2) << r1.data[6] - r2.data[6], r1.data[7] - r2.data[7], r1.data[8] - r2.data[8];
            b.row(3 * count)     << t1.data[0] - t2.data[0];
            b.row(3 * count + 1) << t1.data[1] - t2.data[1];
            b.row(3 * count + 2) << t1.data[2] - t2.data[2];
            ++count;
        }
    }
    auto x = (A.transpose() * A).inverse() * A.transpose() * b;
    KDL::Frame tool2ee(KDL::Rotation::Identity(), KDL::Vector(x(0), x(1), x(2)));
    return tool2ee.Inverse();
}

void KinematicsPlugin::resetSolver()
{
    kdl_chain_ = default_kdl_chain_;
    kdl_chain_.addSegment(tool_segment_);
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    ik_velocity_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(kdl_chain_);
    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(kdl_chain_, joints_limit_min_, joints_limit_max_, *fk_solver_, *ik_velocity_solver_, 200, 1e-5);
}

KDL::Frame KinematicsPlugin::pose2KDLFrame(geometry_msgs::msg::Pose pose) {
    KDL::Frame res;
    res.p.x(pose.position.x);
    res.p.y(pose.position.y);
    res.p.z(pose.position.z);
    res.M = KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    return res;
}

geometry_msgs::msg::Pose KinematicsPlugin::kdlFrame2Pose(KDL::Frame frame)
{
    geometry_msgs::msg::Pose res;
    res.position.x = frame.p.x();
    res.position.y = frame.p.y();
    res.position.z = frame.p.z();
    frame.M.GetQuaternion(res.orientation.x, res.orientation.y, res.orientation.z, res.orientation.w);
    
    return res;
}
