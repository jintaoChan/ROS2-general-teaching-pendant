#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "i_kinematics_solver.h"
#include "i_dynamics_service.h"
#include "robot_handle_ports_adapter.h"

struct RuntimeBootstrapContext {
    rclcpp::Node::SharedPtr robot_handle_node;
    rclcpp::Node::SharedPtr motion_plugin_node;
    std::unique_ptr<RobotHandle> robot_handle;
    std::shared_ptr<RobotHandlePortsAdapter> robot_ports_adapter;
    std::unique_ptr<IKinematicsSolver> kinematics_solver;
    std::unique_ptr<IDynamicsService> dynamics_service;
};

RuntimeBootstrapContext initializeRuntime(
    rclcpp::executors::MultiThreadedExecutor& executor,
    const rclcpp::NodeOptions& node_options);

void shutdownRuntime(RuntimeBootstrapContext& context);
