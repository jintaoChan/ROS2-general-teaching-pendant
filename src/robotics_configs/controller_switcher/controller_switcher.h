#ifndef CONTROLLER_SWITCHER_H
#define CONTROLLER_SWITCHER_H

#include <rclcpp/rclcpp.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include "singleton.hpp"
class ControllerSwitcher : public Singleton<ControllerSwitcher> {

    friend class Singleton<ControllerSwitcher>;
public:
    ControllerSwitcher(const rclcpp::Node::SharedPtr& node);
    ~ControllerSwitcher() = default;

public:
    void switchToControlPad() const;
    void switchToTaskExecutor() const;

private:
    void switchController(std::vector<std::string> activates = std::vector<std::string>{}) const;
    std::vector<std::string> listActiveController() const;


private:
    rclcpp::Node::SharedPtr m_Node;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr m_ControllerSwitcher;
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr m_ControllerListRequester;
    const std::string CONTROL_PAD_CONTROLLER_NAME = "control_pad_controller";
    const std::vector<std::string> TASK_EXECUTOR_CONTROLLER_NAME = {"store_manipulator_controller", "store_gripper_controller"};
};

#endif // CONTROLLER_SWITCHER_H
