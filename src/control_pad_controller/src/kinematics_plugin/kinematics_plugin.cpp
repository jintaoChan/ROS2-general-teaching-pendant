#include "kinematics_plugin.h"
#include <Eigen/Dense>
#include <Eigen/SVD>
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

    // Initialize incremental jog state: desired_pose_ from current FK
    const auto joints_value_map = state_port_->getCurrentJointPosition();
    KDL::JntArray q(joints_num_);
    for (size_t i = 0; i < joints_num_; ++i) {
        q(i) = joints_value_map.at(joints_names_[i]).joint_value;
    }
    fk_solver_->JntToCart(q, desired_pose_);
    last_compute_time_ = state_port_->getTime();
}

std::optional<JointsPosition> KinematicsPlugin::computeCartesianJogCommand(const rclcpp::Time& current_time)
{
    const auto joints_value_map = state_port_->getCurrentJointPosition();
    for (size_t i = 0; i < joints_num_; ++i) {
        current_joint_position_(i) = joints_value_map.at(joints_names_[i]).joint_value;
    }

    // Incremental dt from last compute cycle (not total elapsed time)
    const double dt = current_time.seconds() - last_compute_time_.seconds();
    if (dt <= 0.0) {
        // No time elapsed, return current position to hold steady
        JointsPosition hold;
        for (size_t i = 0; i < joints_num_; ++i) {
            hold[joints_names_[i]].joint_value = current_joint_position_(i);
        }
        return hold;
    }
    last_compute_time_ = current_time;

    // Compute current FK
    KDL::Frame current_fk;
    fk_solver_->JntToCart(current_joint_position_, current_fk);

    // Transform twist into base frame
    auto transformed_linear_vel = selected_coord_system_ * KDL::Vector(twist_msg_.linear.x, twist_msg_.linear.y, twist_msg_.linear.z);
    auto transformed_angular_vel = selected_coord_system_ * KDL::Vector(twist_msg_.angular.x, twist_msg_.angular.y, twist_msg_.angular.z);

    // Build 6D cartesian delta for direction-aware singularity check
    Eigen::Matrix<double, 6, 1> cartesian_delta;
    cartesian_delta << transformed_linear_vel(0) * dt,
                       transformed_linear_vel(1) * dt,
                       transformed_linear_vel(2) * dt,
                       transformed_angular_vel(0) * dt,
                       transformed_angular_vel(1) * dt,
                       transformed_angular_vel(2) * dt;

    // Compute singularity velocity scale (direction-aware)
    const double singularity_scale = computeSingularityScale(current_joint_position_, cartesian_delta);
    if (singularity_scale <= 0.0) {
        desired_pose_ = current_fk;
        JointsPosition hold;
        for (size_t i = 0; i < joints_num_; ++i) {
            hold[joints_names_[i]].joint_value = current_joint_position_(i);
        }
        return hold;
    }

    // IMPORTANT: scale the Cartesian step itself before IK.
    // If we clamp only the joint-space result after IK, the commanded joints no longer
    // correspond to the desired Cartesian target, which can cause oscillation near singularities.
    const KDL::Frame desired_pose_start = desired_pose_;
    KDL::JntArray target_joints_value(joints_num_);
    double step_scale = singularity_scale;
    bool is_ok = false;

    while (step_scale >= kMinCartesianStepScale) {
        KDL::Frame candidate_pose = desired_pose_start;

        double vx = transformed_linear_vel(0) * step_scale;
        double vy = transformed_linear_vel(1) * step_scale;
        double vz = transformed_linear_vel(2) * step_scale;
        double wx = transformed_angular_vel(0) * step_scale;
        double wy = transformed_angular_vel(1) * step_scale;
        double wz = transformed_angular_vel(2) * step_scale;

        candidate_pose.p.x(candidate_pose.p.x() + dt * vx);
        candidate_pose.p.y(candidate_pose.p.y() + dt * vy);
        candidate_pose.p.z(candidate_pose.p.z() + dt * vz);
        candidate_pose.M = selected_coord_system_ *
                           KDL::Rotation::RotX(dt * wx) *
                           KDL::Rotation::RotY(dt * wy) *
                           KDL::Rotation::RotZ(dt * wz) *
                           selected_coord_system_.Inverse() * candidate_pose.M;

        auto ik_ret = ik_solver_->CartToJnt(current_joint_position_, candidate_pose, target_joints_value);
        if (ik_ret < 0) {
            step_scale *= 0.5;
            continue;
        }

        double max_velocity_ratio = 0.0;
        for (size_t i = 0; i < joints_num_; ++i) {
            double joint_vel = std::abs(target_joints_value(i) - current_joint_position_(i)) / dt;
            double vel_limit = kJogJointVelocityScale * state_port_->getJointVelocityLimit(joints_names_[i]);
            if (vel_limit > 0.0) {
                max_velocity_ratio = std::max(max_velocity_ratio, joint_vel / vel_limit);
            }
        }

        if (max_velocity_ratio > 1.0) {
            step_scale /= max_velocity_ratio;
            continue;
        }

        desired_pose_ = candidate_pose;
        is_ok = true;
        break;
    }

    if (!is_ok) {
        desired_pose_ = current_fk;
        JointsPosition hold;
        for (size_t i = 0; i < joints_num_; ++i) {
            hold[joints_names_[i]].joint_value = current_joint_position_(i);
        }
        return hold;
    }

    JointsPosition target_joint_positions;
    for (size_t i = 0; i < joints_num_; ++i) {
        target_joint_positions[joints_names_[i]].joint_value = target_joints_value(i);
    }
    return target_joint_positions;
}

double KinematicsPlugin::computeSingularityScale(const KDL::JntArray& joint_pos,
                                                  const Eigen::Matrix<double, 6, 1>& cartesian_delta)
{
    KDL::Jacobian jac(joints_num_);
    jac_solver_->JntToJac(joint_pos, jac);

    // SVD of the 6×N Jacobian
    const int dims = std::min<int>(jac.data.rows(), jac.data.cols());
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jac.data, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto& sv = svd.singularValues();
    if (dims == 0 || sv(dims - 1) < 1e-10) {
        return 0.0;
    }

    const double condition_number = sv(0) / sv(dims - 1);
    if (condition_number < kLowerSingularityThreshold) {
        return 1.0;
    }

    // Direction awareness: is the commanded motion moving toward or away from singularity?
    // The singular vector for the least singular value = last column of U.
    Eigen::VectorXd vector_towards_singularity = svd.matrixU().col(dims - 1);

    // Verify direction by taking a small step: if condition number increases,
    // vector_towards_singularity points toward singularity; otherwise flip it.
    const Eigen::MatrixXd pseudo_inverse = svd.matrixV() * sv.asDiagonal().inverse() * svd.matrixU().transpose();
    Eigen::VectorXd next_q(joints_num_);
    for (size_t i = 0; i < joints_num_; ++i) {
        next_q(i) = joint_pos(i);
    }
    next_q += pseudo_inverse * (vector_towards_singularity * 0.01);

    // Compute condition number at the test position
    KDL::JntArray test_q(joints_num_);
    for (size_t i = 0; i < joints_num_; ++i) {
        test_q(i) = next_q(i);
    }
    KDL::Jacobian test_jac(joints_num_);
    jac_solver_->JntToJac(test_q, test_jac);
    Eigen::JacobiSVD<Eigen::MatrixXd> test_svd(test_jac.data, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto& test_sv = test_svd.singularValues();
    const double test_condition = (test_sv(dims - 1) > 1e-10) ? test_sv(0) / test_sv(dims - 1) : 1e6;

    if (test_condition <= condition_number) {
        // Our guess was wrong, flip the direction
        vector_towards_singularity *= -1;
    }

    // Check if commanded motion is toward or away from singularity
    const bool moving_towards = vector_towards_singularity.dot(cartesian_delta) > 0;

    // Upper threshold depends on direction: more lenient when leaving singularity
    double upper_threshold;
    if (moving_towards) {
        upper_threshold = kHardStopSingularityThreshold;
    } else {
        const double range = kHardStopSingularityThreshold - kLowerSingularityThreshold;
        upper_threshold = kLowerSingularityThreshold + range * kLeavingSingularityMultiplier;
    }

    if (condition_number >= upper_threshold) {
        return moving_towards ? 0.0 : 1.0;
    }

    double scale = 1.0 - (condition_number - kLowerSingularityThreshold) /
                         (upper_threshold - kLowerSingularityThreshold);
    return std::clamp(scale, 0.0, 1.0);
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
    ik_velocity_solver_ = std::make_unique<KDL::ChainIkSolverVel_wdls>(kdl_chain_);
    ik_velocity_solver_->setLambda(kDlsLambda);
    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(kdl_chain_, joints_limit_min_, joints_limit_max_, *fk_solver_, *ik_velocity_solver_, 200, 1e-5);
    jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_);
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
