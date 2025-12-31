#include <QApplication>
#include <main_window.h>
#include <rclcpp/rclcpp.hpp>
#include "robot_handle.h"
#include "kinematics_plugin.h"
#include "dynamic_plugin.h"
#include "point_pool.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);


    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto robot_handle_node = rclcpp::Node::make_shared("robot_handle", node_options);
    auto kinematics_plugin_node = rclcpp::Node::make_shared("kinematics_plugin", node_options);
    auto controller_switcher_node = rclcpp::Node::make_shared("controller_switcher", node_options);
    
    
    rclcpp::executors::MultiThreadedExecutor main_exec;
    main_exec.add_node(robot_handle_node);
    main_exec.add_node(kinematics_plugin_node);
    std::thread main_exec_thread([&] { main_exec.spin(); });
    // ControllerSwitcher::init(controller_switcher_node); // it spin itself
    RobotHandle::init(robot_handle_node);
    KinematicsPlugin::init(kinematics_plugin_node);
    PointPool::init();

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    a.exec();

    rclcpp::shutdown();
    RobotHandle::destroy();
    KinematicsPlugin::destroy();
    main_exec_thread.join();
    return 0;
}
