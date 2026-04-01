#include "streaming_controller.h"
#include "motion_state_machine.h"

#include <algorithm>
#include <magic_enum/magic_enum.hpp>

StreamingController::StreamingController(
    const rclcpp::Node::SharedPtr& node,
    IRobotStateProvider* state_port,
    IRobotCommandPort* command_port,
    IKinematicsSolver* solver,
    IDynamicsService* dynamics,
    MotionStateMachine& state_machine)
    : node_(node), state_port_(state_port), command_port_(command_port),
      solver_(solver), dynamics_(dynamics), state_machine_(state_machine)
{
    const auto period = std::chrono::nanoseconds(state_port_->getControllerUpdatePeriod());

    cartesian_jog_timer_ = node_->create_wall_timer(
        period, std::bind(&StreamingController::cartesianJogCallback, this));
    cartesian_jog_timer_->cancel();

    joint_jog_timer_ = node_->create_wall_timer(
        period, std::bind(&StreamingController::jointJogCallback, this));
    joint_jog_timer_->cancel();

    drag_timer_ = node_->create_wall_timer(
        period, std::bind(&StreamingController::dragCallback, this));
    drag_timer_->cancel();
}

// ---------------------------------------------------------------------------
// Cartesian jog
// ---------------------------------------------------------------------------

void StreamingController::jogCartesian(const std::array<double, 6>& velocity_arr)
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x = velocity_arr[0];
    msg.linear.y = velocity_arr[1];
    msg.linear.z = velocity_arr[2];
    msg.angular.x = velocity_arr[3];
    msg.angular.y = velocity_arr[4];
    msg.angular.z = velocity_arr[5];
    jogCartesian(msg);
}

void StreamingController::jogCartesian(const geometry_msgs::msg::Twist& twist)
{
    static const geometry_msgs::msg::Vector3 zero_vec{};
    if (twist.angular == zero_vec && twist.linear == zero_vec) {
        cartesian_jog_timer_->cancel();
        if (state_machine_.currentState() == MotionState::CartesianJog) {
            state_machine_.forceIdle();
        }
        return;
    }

    if (!state_machine_.tryTransition(MotionState::CartesianJog)) {
        RCLCPP_WARN(node_->get_logger(), "Cannot start cartesian jog: motion state is %s",
            std::string(magic_enum::enum_name(state_machine_.currentState())).c_str());
        return;
    }

    solver_->prepareCartesianJog(twist);
    cartesian_jog_timer_->reset();
}

// ---------------------------------------------------------------------------
// Joint jog (incremental – no cached target)
// ---------------------------------------------------------------------------

void StreamingController::jogJoint(const std::string& joint_name, double direction, double velo_ratio)
{
    if (!state_machine_.tryTransition(MotionState::JointJog)) {
        RCLCPP_WARN(node_->get_logger(), "Cannot start joint jog: motion state is %s",
            std::string(magic_enum::enum_name(state_machine_.currentState())).c_str());
        return;
    }

    {
        std::lock_guard<std::mutex> lock(joint_jog_mutex_);
        jog_joint_name_ = joint_name;
        jog_direction_ = direction;
        jog_velo_ratio_ = velo_ratio;
        const auto current = state_port_->getCurrentJointPosition();
        auto it = current.find(joint_name);
        if (it != current.end()) {
            jog_commanded_pos_ = it->second.joint_value;
        }
    }
    joint_jog_timer_->reset();
}

// ---------------------------------------------------------------------------
// Drag mode
// ---------------------------------------------------------------------------

void StreamingController::startDragging()
{
    if (!state_machine_.tryTransition(MotionState::Dragging)) {
        RCLCPP_WARN(node_->get_logger(), "Cannot start dragging: motion state is %s",
            std::string(magic_enum::enum_name(state_machine_.currentState())).c_str());
        return;
    }

    command_port_->switchToCST();
    has_pending_drag_task_ = true;
    drag_timer_->reset();
}

void StreamingController::stopDragging()
{
    command_port_->switchToCSP();
    has_pending_drag_task_ = false;
    drag_timer_->cancel();
    state_machine_.forceIdle();
}

bool StreamingController::isDragging() const
{
    return state_machine_.currentState() == MotionState::Dragging;
}

// ---------------------------------------------------------------------------
// Stop helpers
// ---------------------------------------------------------------------------

void StreamingController::stopJog()
{
    cartesian_jog_timer_->cancel();
    joint_jog_timer_->cancel();
    auto state = state_machine_.currentState();
    if (state == MotionState::CartesianJog || state == MotionState::JointJog) {
        state_machine_.forceIdle();
    }
}

void StreamingController::stop()
{
    cartesian_jog_timer_->cancel();
    joint_jog_timer_->cancel();
    if (state_machine_.currentState() == MotionState::Dragging) {
        command_port_->switchToCSP();
        has_pending_drag_task_ = false;
        drag_timer_->cancel();
    }
    state_machine_.forceIdle();
}

// ---------------------------------------------------------------------------
// Timer callbacks
// ---------------------------------------------------------------------------

void StreamingController::cartesianJogCallback()
{
    has_pending_cartesian_jog_task_ = true;
    processCartesianJog();
}

void StreamingController::processCartesianJog()
{
    if (!has_pending_cartesian_jog_task_) return;

    std::lock_guard<std::mutex> lock(cartesian_jog_mutex_);
    has_pending_cartesian_jog_task_ = false;

    const auto target = solver_->computeCartesianJogCommand(state_port_->getTime());
    if (target.has_value()) {
        command_port_->moveJointByAbsPosition(target.value(), 1.0);
    } else {
        RCLCPP_WARN(node_->get_logger(),
            "Cartesian jog IK failed. The robot may be at a singularity or kinematic limit.");
        command_port_->moveJointByAbsPosition(state_port_->getCurrentJointPosition(), 1.0);
    }
}

void StreamingController::jointJogCallback()
{
    has_pending_joint_jog_task_ = true;
    processJointJog();
}

void StreamingController::processJointJog()
{
    if (!has_pending_joint_jog_task_) return;

    std::lock_guard<std::mutex> lock(joint_jog_mutex_);
    has_pending_joint_jog_task_ = false;

    const double dt = state_port_->getControllerUpdatePeriod() * 1e-9;
    const double velocity = jog_direction_ * jog_velo_ratio_
        * state_port_->getJointVelocityLimit(jog_joint_name_);


    double next_pos = jog_commanded_pos_ + velocity * dt;

    const double lower = state_port_->getJointLowerLimit(jog_joint_name_);
    const double upper = state_port_->getJointUpperLimit(jog_joint_name_);
    next_pos = std::clamp(next_pos, lower, upper);

    jog_commanded_pos_ = next_pos;

    JointsPosition cmd;
    cmd[jog_joint_name_].joint_value = next_pos;
    command_port_->moveJointByAbsPosition(cmd, 1.0);

    if (next_pos <= lower || next_pos >= upper) {
        joint_jog_timer_->cancel();
        state_machine_.forceIdle();
    }
}

void StreamingController::dragCallback()
{
    has_pending_drag_task_ = true;
    processDrag();
}

void StreamingController::processDrag()
{
    if (!has_pending_drag_task_) return;

    std::lock_guard<std::mutex> lock(drag_mutex_);
    has_pending_drag_task_ = false;

    static std::once_flag init_flag;
    static JointsAcceleration zero_acc;
    std::call_once(init_flag, [&]() {
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
        RCLCPP_WARN(node_->get_logger(), "RNEA error while dragging! Stopping drag.");
        stopDragging();
    }
}
