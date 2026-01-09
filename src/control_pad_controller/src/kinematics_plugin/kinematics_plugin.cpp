#include <chrono>
#include "kinematics_plugin.h"
#include "database.h"
#include "dynamic_plugin.h"
#include <Eigen/Dense>


using namespace std::chrono_literals;
using namespace std::chrono;

KinematicsPlugin::KinematicsPlugin(const rclcpp::Node::SharedPtr& node)
    :node_(node)
{
    urdf_model_ = RobotHandle::instance().getURDFModel();
    default_kdl_chain_ = RobotHandle::instance().getKDLChain();
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
    const auto& update_period = RobotHandle::instance().getControllerUpdatePeriod();
    cartesian_jog_timer_ = node_->create_wall_timer(nanoseconds(update_period), std::bind(&KinematicsPlugin::cartesianJogCallback, this));
    cartesian_jog_timer_->cancel();

    joint_jog_timer_ = node_->create_wall_timer(nanoseconds(update_period), std::bind(&KinematicsPlugin::jointJogCallback, this));
    joint_jog_timer_->cancel();

    drag_timer_ = node_->create_wall_timer(nanoseconds(update_period), std::bind(&KinematicsPlugin::dragCallback, this));
    drag_timer_->cancel();

    twist_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>("control_pad_controller/remote_cartesian_jog", 10, std::bind(&KinematicsPlugin::remoteCartesianJogCallback, this, std::placeholders::_1));
    joint_sub_ = node_->create_subscription<control_msgs::msg::JointJog>("control_pad_controller/remote_joint_jog", 10, std::bind(&KinematicsPlugin::remoteJointJogCallback, this, std::placeholders::_1));
    pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("control_pad_controller/remote_catesian_move", 10, std::bind(&KinematicsPlugin::remoteCartesianMoveCallback, this, std::placeholders::_1));
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("control_pad_controller/cartesian_pose", 10);
    worker_thread_ = std::thread([this]() {
        while (!stop_worker_) {
            if (
                (has_pending_cartesian_jog_task_ && has_pending_joint_jog_task_) ||
                (has_pending_drag_task_ && has_pending_cartesian_jog_task_) ||
                (has_pending_drag_task_ && has_pending_joint_jog_task_)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                has_pending_cartesian_jog_task_ = false;
                has_pending_joint_jog_task_ = false;
                has_pending_drag_task_ = false;
                joint_jog_timer_->cancel();
                cartesian_jog_timer_->cancel();
                drag_timer_->cancel();
                RCLCPP_WARN(node_->get_logger(), "Multiple movement command at the same time is not allowed!");
            }
            else if (has_pending_cartesian_jog_task_) {
                std::lock_guard<std::mutex> lock(worker_mutex_);
                has_pending_cartesian_jog_task_ = false;
                this->processCartesianJog(); 
            }
            else if (has_pending_joint_jog_task_) {
                std::lock_guard<std::mutex> lock(worker_mutex_);
                has_pending_joint_jog_task_ = false;
                this->processJointJog(); 
            }
            else if(has_pending_drag_task_) {
                std::lock_guard<std::mutex> lock(worker_mutex_);
                has_pending_drag_task_ = false;
                this->processDrag();
            }
            else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
    }
    });
}

KinematicsPlugin::~KinematicsPlugin()
{
    stop_worker_ = true;
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

void KinematicsPlugin::twistRobot(const std::array<double, 6> &velocity_arr) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = velocity_arr[0];
    msg.linear.y = velocity_arr[1];
    msg.linear.z = velocity_arr[2];
    msg.angular.x = velocity_arr[3];
    msg.angular.y = velocity_arr[4];
    msg.angular.z = velocity_arr[5];
    twistRobot(msg);
}

void KinematicsPlugin::twistRobot(const geometry_msgs::msg::Twist &twist_msg) {
    static const geometry_msgs::msg::Vector3 zero_vec{};
    if (twist_msg.angular == zero_vec && twist_msg.linear == zero_vec) {
        cartesian_jog_timer_->cancel();
    }
    else {
        twist_msg_ = twist_msg;
        stored_pose_ = current_pose_;
        switch(coord_sys_type_) {
            case ControlCoordinateSystemType::Tool:{
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
        cartesian_jog_timer_->reset();
    }
}

void KinematicsPlugin::moveJointPositionRelatively(const JointsPosition &pos, double velo_ratio)
{
    JointsPosition absolute_pos;
    for(const auto& p: pos) {
        auto idx = std::distance(joints_names_.begin(), std::find(joints_names_.begin(), joints_names_.end(), p.first));
        absolute_pos[p.first].joint_value = p.second.joint_value + current_joint_position_(idx);
    }
    moveJointPositionAbsolutely(absolute_pos, velo_ratio);
}

void KinematicsPlugin::moveJointPositionAbsolutely(const JointsPosition &pos, double velo_ratio)
{
    velo_ratio_ = velo_ratio;
    stored_joint_position_.header.stamp = RobotHandle::instance().getTime();
    stored_joint_position_.name.clear();
    target_joint_position_.name.clear();
    stored_joint_position_.position.clear();
    target_joint_position_.position.clear();
    for(const auto& p: pos) {
        auto it = std::find(joints_names_.begin(), joints_names_.end(), p.first);
        if(it != joints_names_.end()) {
            target_joint_position_.position.push_back(p.second.joint_value);
            target_joint_position_.name.push_back(p.first);
            auto idx = std::distance(joints_names_.begin(), it);
            stored_joint_position_.position.push_back(current_joint_position_(idx));
            stored_joint_position_.name.push_back(p.first);
        }
    }
    joint_jog_timer_->reset();
}

void KinematicsPlugin::moveToPose(const KDL::Frame &pose, double velo_ratio)
{
    KDL::JntArray target_joints_value(joints_num_);
    auto ik_ret = ik_solver_->CartToJnt(current_joint_position_, pose, target_joints_value);
    JointsPosition target_joint_positions;
    if (ik_ret < 0) {
        RCLCPP_ERROR(node_->get_logger(), "Robot cannot arrive this target");
        throw(std::runtime_error("Robot cannot arrive this target"));
    }
    else{
        for (size_t i = 0; i < joints_num_; ++i) {
            target_joint_positions[joints_names_[i]].joint_value = target_joints_value(i);
        }
        moveJointPositionAbsolutely(target_joint_positions,velo_ratio);
    }
}

void KinematicsPlugin::stop(){
    joint_jog_timer_->cancel();
    cartesian_jog_timer_->cancel();
}

geometry_msgs::msg::PoseStamped KinematicsPlugin::getCurrentCartesianPose()
{
    KDL::Frame pose;
    auto joints_value_map = RobotHandle::instance().getCurrentJointPosition();
    for (size_t i = 0;i < joints_num_; ++i) {
        current_joint_position_(i) = joints_value_map[joints_names_[i]].joint_value;
    }
    fk_solver_->JntToCart(current_joint_position_, pose);

    current_pose_.pose.position.x  = pose.p.x();
    current_pose_.pose.position.y  = pose.p.y();
    current_pose_.pose.position.z  = pose.p.z();
    pose.M.GetQuaternion(current_pose_.pose.orientation.x, current_pose_.pose.orientation.y, current_pose_.pose.orientation.z, current_pose_.pose.orientation.w);
    current_pose_.header.stamp = RobotHandle::instance().getTime();
    pose_pub_->publish(current_pose_);
    return current_pose_;
}

void KinematicsPlugin::selectCoordinateSystem(const ControlCoordinateSystemType& coord_sys)
{
    coord_sys_type_ = coord_sys;
    switch(coord_sys_type_) {
    case ControlCoordinateSystemType::Tool:{
        auto current_tool_frame = RobotHandle::instance().getCurrentToolFrame();
        tool_segment_.setFrameToTip(RobotHandle::instance().getRobotArmToolInfo().at(current_tool_frame));
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

void KinematicsPlugin::cartesianJogCallback()
{
    has_pending_cartesian_jog_task_ = true;
}

void KinematicsPlugin::jointJogCallback()
{
    has_pending_joint_jog_task_ = true;
}

void KinematicsPlugin::dragCallback()
{
    has_pending_drag_task_ = true;
}

void KinematicsPlugin::processCartesianJog()
{
    auto current_time = RobotHandle::instance().getTime();
    auto time_diff = current_time.seconds()  - ((double)stored_pose_.header.stamp.sec + 1e-9 * (double)stored_pose_.header.stamp.nanosec);
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
            RobotHandle::instance().moveJointByAbsPosition(RobotHandle::instance().getCurrentJointPosition(), velo_ratio_);
            cartesian_jog_timer_->cancel();
            RCLCPP_WARN(node_->get_logger(), "Arrived singularity! Emergency stop!");
            return;
        } else {
            bool is_velocity_valid{true};
            for (size_t i = 0; i < joints_num_; ++i) {
                if (std::abs(target_joints_value(i) - current_joint_position_(i)) > RobotHandle::instance().getJointVelocityLimit(joints_names_[i]) * time_diff) {
                    is_velocity_valid = false;
                    break;
                }
            }
            if (!is_velocity_valid) {
                velocity_divider *= 2;
                continue;
            } else {
                is_ok = true;
            }
        }
    }

    if (is_ok) {
        JointsPosition target_joint_positions;
        for (size_t i = 0; i < joints_num_; ++i) {
            target_joint_positions[joints_names_[i]].joint_value = target_joints_value(i);
        }
        RobotHandle::instance().moveJointByAbsPosition(target_joint_positions, velo_ratio_);
    } else {
        RCLCPP_WARN(node_->get_logger(), "The robot is near a singularity. Joint velocities too high. Try reducing speed.");
        RobotHandle::instance().moveJointByAbsPosition(RobotHandle::instance().getCurrentJointPosition(), velo_ratio_);
        cartesian_jog_timer_->cancel();
    }
}

void KinematicsPlugin::processJointJog()
{
    auto current_time = RobotHandle::instance().getTime();
    auto time_diff = current_time.seconds()  - ((double)stored_joint_position_.header.stamp.sec + 1e-9 * (double)stored_joint_position_.header.stamp.nanosec);
    std::vector<bool> joint_arrived(target_joint_position_.name.size(), false);
    
    JointsPosition position_to_send;
    for(size_t i = 0; i < target_joint_position_.name.size(); ++i) {
        auto name = target_joint_position_.name[i];
        auto velocity = velo_ratio_ * RobotHandle::instance().getJointVelocityLimit(name);
        auto total_time = std::abs((target_joint_position_.position[i] - stored_joint_position_.position[i]) / velocity);
        double sign = target_joint_position_.position[i] > stored_joint_position_.position[i] ? 1 : -1;
        velocity *= sign;
        if(time_diff < total_time) {
            position_to_send[name].joint_value = stored_joint_position_.position[i] + velocity * time_diff;
        }
        else {
            position_to_send[name].joint_value = target_joint_position_.position[i];
            joint_arrived[i] = true;
        }
    }
    if(std::all_of(joint_arrived.begin(), joint_arrived.end(), [](const auto& v) { return v == true;})) {
        joint_jog_timer_->cancel();
    }
    RobotHandle::instance().moveJointByAbsPosition(position_to_send, velo_ratio_);

}

void KinematicsPlugin::processDrag()
{
    static std::once_flag initialize_zero_acc_flag;
    static JointsAcceleration zero_acc;
    std::call_once(initialize_zero_acc_flag, [&](){
        for(const auto& n : RobotHandle::instance().getJointsName()) {
            zero_acc[n].joint_value = 0;
        }
    });
    auto res = DynamicPlugin::instance().rnea(RobotHandle::instance().getCurrentJointPosition(),
                                              RobotHandle::instance().getCurrentJointVelocity(),
                                              zero_acc);

    if(res.has_value()) {
        RobotHandle::instance().setJointTorque(res.value());
    }
    else {
        RCLCPP_WARN(node_->get_logger(), "Error while dragging! Stop dragging");
        this->stopDragging();
    }

}

void KinematicsPlugin::remoteCartesianJogCallback(const geometry_msgs::msg::TwistStamped& msg)
{
    auto modulated_msg = msg.twist;
    modulated_msg.linear.x = std::min(modulated_msg.linear.x, 1.0);
    modulated_msg.linear.x = std::max(modulated_msg.linear.x, -1.0);
    modulated_msg.linear.x *=  RobotHandle::instance().getCartesianLimitsMaxTransVel();
    modulated_msg.linear.y = std::min(modulated_msg.linear.y, 1.0);
    modulated_msg.linear.y = std::max(modulated_msg.linear.y, -1.0);
    modulated_msg.linear.y *=  RobotHandle::instance().getCartesianLimitsMaxTransVel();
    modulated_msg.linear.z = std::min(modulated_msg.linear.z, 1.0);
    modulated_msg.linear.z = std::max(modulated_msg.linear.z, -1.0);
    modulated_msg.linear.z *=  RobotHandle::instance().getCartesianLimitsMaxTransVel();
    modulated_msg.angular.x = std::min(modulated_msg.angular.x, 1.0);
    modulated_msg.angular.x = std::max(modulated_msg.angular.x, -1.0);
    modulated_msg.angular.x *=  RobotHandle::instance().getCartesianLimitsMaxRotVel();
    modulated_msg.angular.y = std::min(modulated_msg.angular.y, 1.0);
    modulated_msg.angular.y = std::max(modulated_msg.angular.y, -1.0);
    modulated_msg.angular.y *=  RobotHandle::instance().getCartesianLimitsMaxRotVel();
    modulated_msg.angular.z = std::min(modulated_msg.angular.z, 1.0);
    modulated_msg.angular.z = std::max(modulated_msg.angular.z, -1.0);
    modulated_msg.angular.z *=  RobotHandle::instance().getCartesianLimitsMaxRotVel();
    twistRobot(modulated_msg);
}

void KinematicsPlugin::remoteJointJogCallback(const control_msgs::msg::JointJog& msg)
{
    JointsVelocity vel;
    for(size_t i = 0; i < msg.joint_names.size(); ++i) {
        auto joint_name = msg.joint_names[i];
        vel[joint_name].joint_value = msg.velocities[i] * RobotHandle::instance().getJointVelocityLimit(joint_name);
    }
    RobotHandle::instance().moveJointByVelcoity(vel);
}

void KinematicsPlugin::remoteCartesianMoveCallback(const geometry_msgs::msg::PoseStamped &msg)
{
    KDL::Frame target_pose = pose2KDLFrame(msg.pose);
    KDL::JntArray target_joints_value(joints_num_);
    auto ik_ret = ik_solver_->CartToJnt(current_joint_position_, target_pose, target_joints_value);
    if(ik_ret < 0) {
        RCLCPP_WARN(node_->get_logger(), "Invalid pose! Abort!");
    }
    else {
        JointsPosition joint_position;
        for(size_t i = 0; i < joints_names_.size(); ++i) {
            joint_position[joints_names_[i]].joint_value = target_joints_value(i);
        }
        moveJointPositionAbsolutely(joint_position, 1.0);
    }
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
        RCLCPP_ERROR(node_->get_logger(), "TCP calibration need 3 pose at least!");
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

void KinematicsPlugin::startDragging()
{
    RobotHandle::instance().switchToCST();
    has_pending_drag_task_ = true;
    drag_timer_->reset();
}

void KinematicsPlugin::stopDragging()
{
    RobotHandle::instance().switchToCSP();
    has_pending_drag_task_ = false;
    drag_timer_->cancel();
}

bool KinematicsPlugin::isDragging()
{
    return !drag_timer_->is_canceled();
}

bool KinematicsPlugin::isRunning()
{
    return !drag_timer_->is_canceled() || !cartesian_jog_timer_->is_canceled() || !joint_jog_timer_->is_canceled();
}
