#include <QApplication>
#include <main_window.h>
#include <rclcpp/rclcpp.hpp>
#include "robot_description.h"
#include "controller_switcher.h"
#include "mode_changer.h"
#include "cartesian_controller.h"
#include "move_executor.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions nodeOptions;
    // nodeOptions.allow_undeclared_parameters(true);
    nodeOptions.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("auto_store_ui", nodeOptions);
    RobotDescription::init(node);
    ControllerSwitcher::init(node);
    ModeChanger::init(node);
    MoveExecutor::init(node);

    auto ccNode = rclcpp::Node::make_shared("cartesian_controller", nodeOptions);
    CartesianController::init(ccNode);
    std::thread nodeThread([&]() {
        rclcpp::spin(ccNode);
    });
    nodeThread.detach();

    QApplication a(argc, argv);
    MainWindow w;
    w.show();



    return a.exec();
}
