#include "runtime_bootstrap.h"

#include "dynamic_plugin.h"
#include "kinematics_plugin.h"
#include "motion_plugin.h"
#include "robot_handle.h"
#include "io_task.h"

RuntimeBootstrapContext initializeRuntime(
    rclcpp::executors::MultiThreadedExecutor& executor,
    const rclcpp::NodeOptions& node_options)
{
    RuntimeBootstrapContext context;

    context.robot_handle_node = rclcpp::Node::make_shared("robot_handle", node_options);
    context.motion_plugin_node = rclcpp::Node::make_shared("kinematics_plugin", node_options);

    executor.add_node(context.robot_handle_node);
    executor.add_node(context.motion_plugin_node);

    context.robot_handle = std::make_unique<RobotHandle>(context.robot_handle_node);

    IRobotStateProvider* state_port = context.robot_handle.get();
    IRobotCommandPort* command_port = context.robot_handle.get();
    IRobotEvents* event_port = context.robot_handle.get();
    IOTask::configurePorts(command_port, event_port);
    context.dynamics_service = std::make_shared<DynamicPlugin>(state_port);
    context.kinematics_solver = std::make_unique<KinematicsPlugin>(state_port);
    MotionPlugin::init(context.motion_plugin_node, state_port, command_port, context.kinematics_solver.get(), context.dynamics_service.get());
    context.robot_handle->setDynamicsService(context.dynamics_service);

    return context;
}

void shutdownRuntime(RuntimeBootstrapContext& context)
{
    IOTask::configurePorts(nullptr, nullptr);
    if (context.robot_handle) {
        context.robot_handle->setDynamicsService({});
    }
    MotionPlugin::destroy();

    context.robot_handle.reset();
    context.dynamics_service.reset();
    context.kinematics_solver.reset();
    context.motion_plugin_node.reset();
    context.robot_handle_node.reset();
}
