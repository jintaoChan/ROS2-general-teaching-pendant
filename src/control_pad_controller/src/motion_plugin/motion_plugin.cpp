#include "motion_plugin.h"

#include <stdexcept>

MotionPlugin::MotionPlugin(
    const rclcpp::Node::SharedPtr& node,
    IRobotStateProvider* state_port,
    IRobotCommandPort* command_port,
    IKinematicsSolver* solver,
    IDynamicsService* dynamics)
    : node_(node), state_port_(state_port), command_port_(command_port), solver_(solver), dynamics_(dynamics)
{
    if (node_ == nullptr || state_port_ == nullptr || command_port_ == nullptr || solver_ == nullptr || dynamics_ == nullptr) {
        throw std::invalid_argument("MotionPlugin requires non-null node, robot ports, solver, and dynamics service");
    }

    const auto& update_period = state_port_->getControllerUpdatePeriod();
    cartesian_jog_timer_ = node_->create_wall_timer(
        std::chrono::nanoseconds(update_period),
        std::bind(&MotionPlugin::cartesianJogCallback, this));
    cartesian_jog_timer_->cancel();

    joint_jog_timer_ = node_->create_wall_timer(
        std::chrono::nanoseconds(update_period),
        std::bind(&MotionPlugin::jointJogCallback, this));
    joint_jog_timer_->cancel();

    drag_timer_ = node_->create_wall_timer(
        std::chrono::nanoseconds(update_period),
        std::bind(&MotionPlugin::dragCallback, this));
    drag_timer_->cancel();
}

void MotionPlugin::twistRobot(const std::array<double, 6>& velocity_arr)
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x = velocity_arr[0];
    msg.linear.y = velocity_arr[1];
    msg.linear.z = velocity_arr[2];
    msg.angular.x = velocity_arr[3];
    msg.angular.y = velocity_arr[4];
    msg.angular.z = velocity_arr[5];
    twistRobot(msg);
}

void MotionPlugin::twistRobot(const geometry_msgs::msg::Twist& twist_msg)
{
    static const geometry_msgs::msg::Vector3 zero_vec{};
    if (twist_msg.angular == zero_vec && twist_msg.linear == zero_vec) {
        cartesian_jog_timer_->cancel();
        return;
    }

    solver_->prepareCartesianJog(twist_msg);
    cartesian_jog_timer_->reset();
}

void MotionPlugin::moveJointPositionRelatively(const JointsPosition& pos, double velo_ratio)
{
    JointsPosition absolute_pos;
    const auto& current = state_port_->getCurrentJointPosition();
    for (const auto& p : pos) {
        auto it = current.find(p.first);
        if (it == current.end()) {
            continue;
        }
        absolute_pos[p.first].joint_value = p.second.joint_value + it->second.joint_value;
    }
    moveJointPositionAbsolutely(absolute_pos, velo_ratio);
}

void MotionPlugin::moveJointPositionAbsolutely(const JointsPosition& pos, double velo_ratio)
{
    velo_ratio_ = velo_ratio;
    stored_joint_position_.header.stamp = state_port_->getTime();
    stored_joint_position_.name.clear();
    target_joint_position_.name.clear();
    stored_joint_position_.position.clear();
    target_joint_position_.position.clear();

    const auto& current = state_port_->getCurrentJointPosition();
    for (const auto& p : pos) {
        auto it = current.find(p.first);
        if (it == current.end()) {
            continue;
        }
        target_joint_position_.position.push_back(p.second.joint_value);
        target_joint_position_.name.push_back(p.first);
        stored_joint_position_.position.push_back(it->second.joint_value);
        stored_joint_position_.name.push_back(p.first);
    }

    if (!target_joint_position_.name.empty()) {
        joint_jog_timer_->reset();
    }
}

void MotionPlugin::moveToPose(const KDL::Frame& pose, double velo_ratio)
{
    auto target_joint_positions = solver_->solvePoseIK(pose);
    if (!target_joint_positions.has_value()) {
        throw std::runtime_error("Robot cannot arrive this target");
    }
    moveJointPositionAbsolutely(target_joint_positions.value(), velo_ratio);
}

void MotionPlugin::stop()
{
    cartesian_jog_timer_->cancel();
    joint_jog_timer_->cancel();
}

void MotionPlugin::startDragging()
{
    command_port_->switchToCST();
    has_pending_drag_task_ = true;
    drag_timer_->reset();
}

void MotionPlugin::stopDragging()
{
    command_port_->switchToCSP();
    has_pending_drag_task_ = false;
    drag_timer_->cancel();
}

bool MotionPlugin::isDragging()
{
    return drag_timer_ && !drag_timer_->is_canceled();
}

bool MotionPlugin::isRunning()
{
    return isDragging() ||
           (joint_jog_timer_ && !joint_jog_timer_->is_canceled()) ||
           (cartesian_jog_timer_ && !cartesian_jog_timer_->is_canceled());
}

geometry_msgs::msg::PoseStamped MotionPlugin::getCurrentCartesianPose()
{
    return solver_->getCurrentCartesianPose();
}

void MotionPlugin::selectCoordinateSystem(const ControlCoordinateSystemType& coord_sys)
{
    solver_->selectCoordinateSystem(coord_sys);
}

void MotionPlugin::refreshCoordinateSystem()
{
    solver_->refreshCoordinateSystem();
}

KDL::Frame MotionPlugin::tcpCalibration(const MovePointInfo& points)
{
    return solver_->tcpCalibration(points);
}

void MotionPlugin::identify(const size_t& db_start_index, const size_t& db_end_index)
{
    dynamics_->identify(db_start_index, db_end_index);
}

bool MotionPlugin::isDynamicsReady() const
{
    return dynamics_->isReady();
}

const Eigen::MatrixXd& MotionPlugin::getDynamicsBaseParams()
{
    return dynamics_->getBaseParams();
}

const Eigen::MatrixXd& MotionPlugin::getDynamicsFrictionParams()
{
    return dynamics_->getFrictionParams();
}

const Eigen::MatrixXd& MotionPlugin::getDynamicsDepPb()
{
    return dynamics_->getDepPb();
}

const Eigen::MatrixXd& MotionPlugin::getDynamicsDepPd()
{
    return dynamics_->getDepPd();
}

const Eigen::MatrixXd& MotionPlugin::getDynamicsDepKd()
{
    return dynamics_->getDepKd();
}

void MotionPlugin::setDynamicsParams(
    const Eigen::MatrixXd& base, const Eigen::MatrixXd& friction,
    const Eigen::MatrixXd& Pb, const Eigen::MatrixXd& Pd, const Eigen::MatrixXd& Kd)
{
    dynamics_->setParams(base, friction, Pb, Pd, Kd);
}

void MotionPlugin::cartesianJogCallback()
{
    has_pending_cartesian_jog_task_ = true;
    processCartesianJog();
}

void MotionPlugin::processCartesianJog()
{
    if (!has_pending_cartesian_jog_task_) {
        return;
    }

    std::lock_guard<std::mutex> lock(cartesian_jog_mutex_);
    has_pending_cartesian_jog_task_ = false;

    const auto target_joint_positions = solver_->computeCartesianJogCommand(state_port_->getTime());
    if (target_joint_positions.has_value()) {
        command_port_->moveJointByAbsPosition(target_joint_positions.value(), velo_ratio_);
    } else {
        RCLCPP_WARN(node_->get_logger(), "The robot is near a singularity. Joint velocities too high. Try reducing speed.");
        command_port_->moveJointByAbsPosition(state_port_->getCurrentJointPosition(), velo_ratio_);
        cartesian_jog_timer_->cancel();
    }
}

void MotionPlugin::jointJogCallback()
{
    has_pending_joint_jog_task_ = true;
    processJointJog();
}

void MotionPlugin::processJointJog()
{
    if (!has_pending_joint_jog_task_) {
        return;
    }

    std::lock_guard<std::mutex> lock(joint_jog_mutex_);
    has_pending_joint_jog_task_ = false;

    auto current_time = state_port_->getTime();
    auto time_diff = current_time.seconds() -
        (static_cast<double>(stored_joint_position_.header.stamp.sec) +
         1e-9 * static_cast<double>(stored_joint_position_.header.stamp.nanosec));

    std::vector<bool> joint_arrived(target_joint_position_.name.size(), false);
    JointsPosition position_to_send;

    for (size_t i = 0; i < target_joint_position_.name.size(); ++i) {
        const auto& name = target_joint_position_.name[i];
        auto velocity = velo_ratio_ * state_port_->getJointVelocityLimit(name);
        auto total_time = std::abs((target_joint_position_.position[i] - stored_joint_position_.position[i]) / velocity);
        double sign = target_joint_position_.position[i] > stored_joint_position_.position[i] ? 1.0 : -1.0;
        velocity *= sign;

        if (time_diff < total_time) {
            position_to_send[name].joint_value = stored_joint_position_.position[i] + velocity * time_diff;
        } else {
            position_to_send[name].joint_value = target_joint_position_.position[i];
            joint_arrived[i] = true;
        }
    }

    if (std::all_of(joint_arrived.begin(), joint_arrived.end(), [](const auto& v) { return v; })) {
        joint_jog_timer_->cancel();
    }
    command_port_->moveJointByAbsPosition(position_to_send, velo_ratio_);
}

void MotionPlugin::dragCallback()
{
    has_pending_drag_task_ = true;
    processDrag();
}

void MotionPlugin::processDrag()
{
    if (!has_pending_drag_task_) {
        return;
    }

    std::lock_guard<std::mutex> lock(drag_mutex_);
    has_pending_drag_task_ = false;

    static std::once_flag initialize_zero_acc_flag;
    static JointsAcceleration zero_acc;
    std::call_once(initialize_zero_acc_flag, [&]() {
        for (const auto& n : state_port_->getJointsName()) {
            zero_acc[n].joint_value = 0;
        }
    });

    auto res = dynamics_->rnea(
        state_port_->getCurrentJointPosition(),
        state_port_->getCurrentJointVelocity(),
        zero_acc);

    if (res.has_value()) {
        command_port_->setJointTorque(res.value());
    } else {
        RCLCPP_WARN(node_->get_logger(), "Error while dragging! Stop dragging");
        stopDragging();
    }
}
