#include <QApplication>
#include <main_window.h>
#include <rclcpp/rclcpp.hpp>
#include "robot_description.hpp"
#include "controller_switcher.h"
#include "mode_changer.h".h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions nodeOptions;
    nodeOptions.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("auto_store_ui", nodeOptions);
    RobotDescription::init(node);
    ControllerSwitcher::init(node);
    ModeChanger::init(node);


    QApplication a(argc, argv);
    MainWindow w;
    w.show();



    return a.exec();
}
