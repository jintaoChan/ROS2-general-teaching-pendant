#ifndef CONTROLLER_SWITCHER_H
#define CONTROLLER_SWITCHER_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>

#include "singleton.hpp"

class ControllerSwitcher : public Singleton<ControllerSwitcher> {

friend class Singleton<ControllerSwitcher>;

public:
    ControllerSwitcher(const rclcpp::Node::SharedPtr& node);
    ~ControllerSwitcher() = default;

public:
    void switchToControlPad();
    void switchToTaskExecutor();

private:
    void switchController(std::vector<std::string> activates = std::vector<std::string>{});
    std::vector<std::string> listActiveController();


private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_switcher_;
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr controller_list_requester_;
    const std::string CONTROL_PAD_CONTROLLER_NAME = "control_pad_controller";
    std::vector<std::string> MOVE_GROUP_CONTROLLER_NAMES;
    bool switch_success_{false};
    bool switch_ready_{false};
    bool list_ready_{false};
    bool list_success_{false};
};

#endif // CONTROLLER_SWITCHER_H
