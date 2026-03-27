#include <QApplication>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include "main_window.h"
#include "runtime_bootstrap.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);


    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    rclcpp::executors::MultiThreadedExecutor main_exec;
    auto runtime_context = initializeRuntime(main_exec, node_options);
    std::thread main_exec_thread([&] { main_exec.spin(); });

    {
        QApplication a(argc, argv);
        AppPorts app_ports{
            runtime_context.robot_ports_adapter.get(),
            runtime_context.robot_ports_adapter.get(),
            runtime_context.robot_ports_adapter.get()};
        MainWindow w(app_ports);
        w.show();

        QTimer ros_guard;
        QObject::connect(&ros_guard, &QTimer::timeout, [&a]() {
            if (!rclcpp::ok()) {
                a.quit();
            }
        });
        ros_guard.start(100);

        a.exec();
    }

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    shutdownRuntime(runtime_context);
    main_exec.cancel();
    main_exec_thread.join();
    return 0;
}
