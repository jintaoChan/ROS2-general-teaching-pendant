#include <rclcpp/rclcpp.hpp>
#include "robot_handle.h"
#include "controller_switcher.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto robot_handle_node = rclcpp::Node::make_shared("robot_handle", node_options);
    // auto kinematics_plugin_node = rclcpp::Node::make_shared("kinematics_plugin", node_options);
    auto controller_switcher_node = rclcpp::Node::make_shared("controller_switcher", node_options);

    RobotHandle::init(robot_handle_node);
    ControllerSwitcher::init(controller_switcher_node);

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(robot_handle_node);
    // exec.add_node(kinematics_plugin_node);
    exec.add_node(controller_switcher_node);
    exec.spin();
    return 0;
}
