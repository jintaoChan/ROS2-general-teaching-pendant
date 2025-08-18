#include <chrono>
#include "kinematics_plugin.h"


using namespace std::chrono_literals;
using namespace std::chrono;

KinematicsPlugin::KinematicsPlugin(const rclcpp::Node::SharedPtr& node)
    :node_(node)
{
    auto param = AcquireParam<std::string>("/robot_state_publisher", "robot_description");
    urdf_model_.initString(param);
    kdl_parser::treeFromUrdfModel(urdf_model_, kdl_tree_);
    root_frame_id_ = RobotHandle::instance().getFrameID();
    current_frame_id_ = root_frame_id_;
    kdl_tree_.getChain(root_frame_id_, RobotHandle::instance().getRobotArmEndLinkName(), kdl_chain_);
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    ik_velocity_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(kdl_chain_);
    joints_num_ = kdl_chain_.getNrOfJoints();
    current_joint_position_.resize(joints_num_);

    joint_velocity_limit_ = RobotHandle::instance().getJointsVelocityLimits();
    KDL::JntArray joints_limit_min(joints_num_), joints_limit_max(joints_num_);
    for (size_t i = 0, idx = 0; i < kdl_chain_.segments.size(); ++i) {
        if(kdl_chain_.segments[i].getJoint().getType() == KDL::Joint::None) {continue;}
        const auto& joint_name = kdl_chain_.segments[i].getJoint().getName();
        joints_names_.push_back(joint_name);
        auto joint = urdf_model_.getJoint(joint_name);
        if (joint && joint->limits) {
            joints_limit_min(idx) = joint->limits->lower;
            joints_limit_max(idx) = joint->limits->upper;
        }
        else {
            joints_limit_min(idx) = -std::numeric_limits<double>::max();
            joints_limit_max(idx) = std::numeric_limits<double>::max();
        }
        ++idx;
    }
    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(kdl_chain_, joints_limit_min, joints_limit_max, *fk_solver_, *ik_velocity_solver_, 200, 1e-5);
    current_pose_.header.frame_id = root_frame_id_;

    cartesian_jog_timer_ = node_->create_wall_timer(milliseconds(RobotHandle::instance().getControllerUpdatePeriod()), std::bind(&KinematicsPlugin::cartesianJogCallback, this));
    cartesian_jog_timer_->cancel();

    joint_jog_timer_ = node_->create_wall_timer(milliseconds(RobotHandle::instance().getControllerUpdatePeriod()), std::bind(&KinematicsPlugin::jointJogCallback, this));
    joint_jog_timer_->cancel();

    twist_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>("control_pad_controller/remote_cartesian_jog", 10, std::bind(&KinematicsPlugin::remoteCartesianJogCallback, this, std::placeholders::_1));
    joint_sub_ = node_->create_subscription<control_msgs::msg::JointJog>("control_pad_controller/remote_joint_jog", 10, std::bind(&KinematicsPlugin::remoteJointJogCallback, this, std::placeholders::_1));
    pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("control_pad_controller/remote_catesian_move", 10, std::bind(&KinematicsPlugin::remoteCartesianMoveCallback, this, std::placeholders::_1));
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("control_pad_controller/cartesian_pose", 10);
    worker_thread_ = std::thread([this]() {
        while (!stop_worker_) {
            if (has_pending_cartesian_jog_task_ && has_pending_joint_jog_task_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                RCLCPP_WARN(node_->get_logger(), "Twist and jog at the same time is not allowed!");
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
        if(current_frame_id_ == root_frame_id_) {
            selected_coord_system_ = KDL::Rotation::Identity();
        }
        else {
            selected_coord_system_ = KDL::Rotation::Quaternion(stored_pose_.pose.orientation.x, stored_pose_.pose.orientation.y, stored_pose_.pose.orientation.z, stored_pose_.pose.orientation.w);
        }
        cartesian_jog_timer_->reset();
    }
}

void KinematicsPlugin::moveJointPositionRelatively(const JointsPosition &pos)
{
    JointsPosition absolute_pos;
    for(const auto& p: pos) {
        auto idx = std::distance(joints_names_.begin(), std::find(joints_names_.begin(), joints_names_.end(), p.first));
        absolute_pos[p.first].joint_value = p.second.joint_value + current_joint_position_(idx);
    }
    moveJointPositionAbsolutely(absolute_pos);
}

void KinematicsPlugin::moveJointPositionAbsolutely(const JointsPosition &pos)
{
    stored_joint_position_.header.stamp = RobotHandle::instance().getTime();
    stored_joint_position_.name.clear();
    target_joint_position_.name.clear();
    stored_joint_position_.position.clear();
    target_joint_position_.position.clear();
    for(const auto& p: pos) {
        target_joint_position_.position.push_back(p.second.joint_value);
        target_joint_position_.name.push_back(p.first);
        auto idx = std::distance(joints_names_.begin(), std::find(joints_names_.begin(), joints_names_.end(), p.first));
        stored_joint_position_.position.push_back(current_joint_position_(idx));
        stored_joint_position_.name.push_back(p.first);
    }
    joint_jog_timer_->reset();
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
    switch(coord_sys) {
        case ControlCoordinateSystemType::Tool:
            current_frame_id_ = RobotHandle::instance().getRobotArmEndLinkName();
            break;
        case ControlCoordinateSystemType::Base:
            current_frame_id_ = root_frame_id_;
            break;
    }
}

void KinematicsPlugin::cartesianJogCallback()
{
    has_pending_cartesian_jog_task_ = true;
}

void KinematicsPlugin::jointJogCallback()
{
    has_pending_joint_jog_task_ = true;
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
            RobotHandle::instance().moveJointByAbsPosition(RobotHandle::instance().getCurrentJointPosition());
            cartesian_jog_timer_->cancel();
            RCLCPP_WARN(node_->get_logger(), "Arrived singularity! Emergency stop!");
            return;
        } else {
            bool is_velocity_valid{true};
            for (size_t i = 0; i < joints_num_; ++i) {
                if (std::abs(target_joints_value(i) - current_joint_position_(i)) > joint_velocity_limit_[joints_names_[i]].joint_value * time_diff) {
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
        RobotHandle::instance().moveJointByAbsPosition(target_joint_positions);
    } else {
        RCLCPP_WARN(node_->get_logger(), "The robot is near a singularity. Joint velocities too high. Try reducing speed.");
        RobotHandle::instance().moveJointByAbsPosition(RobotHandle::instance().getCurrentJointPosition());
        cartesian_jog_timer_->cancel();
    }
}

void KinematicsPlugin::processJointJog()
{
    auto current_time = RobotHandle::instance().getTime();
    auto time_diff = current_time.seconds()  - ((double)stored_joint_position_.header.stamp.sec + 1e-9 * (double)stored_joint_position_.header.stamp.nanosec);
    const auto& joints_vel_limit = RobotHandle::instance().getJointsVelocityLimits();
    const auto& joints_acc_limit = RobotHandle::instance().getJointsAccelerationLimits();
    const auto& joints_dec_limit = RobotHandle::instance().getJointsDecelerationLimits();
    JointsPosition position_to_send;
    std::vector<bool> joint_arrived(target_joint_position_.name.size(), false);
    for(size_t i = 0; i < target_joint_position_.name.size(); ++i) {
        const auto& joint_name = target_joint_position_.name[i];
        const auto& max_vel = joints_vel_limit.at(joint_name).joint_value;
        const auto& acc = joints_acc_limit.at(joint_name).joint_value;
        const auto& dec = std::abs(joints_dec_limit.at(joint_name).joint_value);
        auto distance = target_joint_position_.position[i] - stored_joint_position_.position[i];
        double sign = distance > 0 ? 1 : -1;
        distance *= sign;
        double time_to_acc = max_vel / acc;
        double time_to_dec = max_vel / dec;
        double distance_to_max_vel = 0.5 * (acc * time_to_acc * time_to_acc + dec * time_to_dec * time_to_dec);
        if(distance < distance_to_max_vel) {
            double max_vel_time = std::sqrt(2 * distance / (acc + acc * acc / dec));
            double total_time = max_vel_time + acc * max_vel_time / dec;
            if(time_diff < max_vel_time) {
                position_to_send[joint_name].joint_value = stored_joint_position_.position[i] + sign * 0.5 * acc * time_diff * time_diff;
            }
            else if (time_diff < total_time) {
                position_to_send[joint_name].joint_value = stored_joint_position_.position[i] + 
                                                           sign * distance - 
                                                           sign * 0.5 * dec * (total_time - time_diff) * (total_time - time_diff);
            }
            else {
                position_to_send[joint_name].joint_value = target_joint_position_.position[i];
                joint_arrived[i] = true;
            }
        }
        else {
            double total_time = (distance - distance_to_max_vel) / max_vel + time_to_dec + time_to_dec;
            if(time_diff < time_to_acc) {
                position_to_send[joint_name].joint_value = stored_joint_position_.position[i] + sign * 0.5 * acc * time_diff * time_diff;
            }
            else if(time_diff < total_time - time_to_dec) {
                position_to_send[joint_name].joint_value = stored_joint_position_.position[i] +
                                                           sign * 0.5 * acc * time_to_acc * time_to_acc +
                                                           sign * max_vel * (time_diff - time_to_acc);
            }
            else if(time_diff < total_time){
                position_to_send[joint_name].joint_value = stored_joint_position_.position[i] + 
                                                           sign * distance - 
                                                           sign * 0.5 * dec * (total_time - time_diff) * (total_time - time_diff);
            }
            else {
                position_to_send[joint_name].joint_value = target_joint_position_.position[i];
                joint_arrived[i] = true;
            }
        }
    }
    if(std::all_of(joint_arrived.begin(), joint_arrived.end(), [](const auto& v) { return v == true;})) {
        joint_jog_timer_->cancel();
    }
    RobotHandle::instance().moveJointByAbsPosition(position_to_send);

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
    const auto& limits = RobotHandle::instance().getJointsVelocityLimits();
    for(size_t i = 0; i < msg.joint_names.size(); ++i) {
        auto joint_name = msg.joint_names[i];
        if(limits.find(joint_name) == limits.end()) {
            RCLCPP_WARN(node_->get_logger(), "Joint name: %s does not exist! Abort!", joint_name.c_str());
            break;
        }
        vel[joint_name].joint_value = msg.velocities[i] * limits.at(joint_name).joint_value;
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
        moveJointPositionAbsolutely(joint_position);
    }
}

KDL::Frame KinematicsPlugin::pose2KDLFrame(geometry_msgs::msg::Pose pose) {
    KDL::Frame res;
    res.p.x(pose.position.x);
    res.p.y(pose.position.y);
    res.p.z(pose.position.z);
    res.M = KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    return res;
}
