#include "runtime_bootstrap.h"

#include "dynamic_plugin.h"
#include "kinematics_plugin.h"
#include "motion_plugin.h"
#include "point_pool.h"
#include "robot_handle.h"
#include "io_task.h"
#include "task_executor.h"

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
    context.robot_ports_adapter = std::make_shared<RobotHandlePortsAdapter>(*context.robot_handle);

    IRobotStateProvider* state_port = context.robot_ports_adapter.get();
    IRobotCommandPort* command_port = context.robot_ports_adapter.get();
    IRobotEvents* event_port = context.robot_ports_adapter.get();
    IOTask::configurePorts(command_port, event_port);
    context.dynamics_service = std::make_unique<DynamicPlugin>(state_port);
    context.kinematics_solver = std::make_unique<KinematicsPlugin>(state_port);
    MotionPlugin::init(context.motion_plugin_node, state_port, command_port, context.kinematics_solver.get(), context.dynamics_service.get());
    context.robot_handle->setDynamicsService(context.dynamics_service.get());

    PointPool::init();
    TaskExecutor::init();

    return context;
}

void shutdownRuntime(RuntimeBootstrapContext& context)
{
    TaskExecutor::destroy();
    PointPool::destroy();
    IOTask::configurePorts(nullptr, nullptr);
    if (context.robot_handle) {
        context.robot_handle->setDynamicsService(nullptr);
    }
    MotionPlugin::destroy();

    context.robot_ports_adapter.reset();
    context.robot_handle.reset();
    context.dynamics_service.reset();
    context.kinematics_solver.reset();
    context.motion_plugin_node.reset();
    context.robot_handle_node.reset();
}
