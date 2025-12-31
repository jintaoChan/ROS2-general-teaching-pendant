#pragma once

#include <rclcpp/rclcpp.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

#include "singleton.hpp"


class ControllerSwitcher : public Singleton<ControllerSwitcher>{
    friend class Singleton<ControllerSwitcher>;
public:
    ControllerSwitcher(const rclcpp::Node::SharedPtr& node);
    ~ControllerSwitcher() = default;

public:
    void switchToJTC();
    void switchToEffect();

private:
    void switchController(const std::vector<std::string>& activate, const std::vector<std::string>& deactivate);


private:
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
    rclcpp::Node::SharedPtr node_;

};