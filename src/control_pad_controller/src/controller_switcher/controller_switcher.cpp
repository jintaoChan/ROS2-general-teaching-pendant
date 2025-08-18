#include "controller_switcher.h"
#include "functional.hpp"
#include "kinematics_plugin.h"

ControllerSwitcher::ControllerSwitcher(const rclcpp::Node::SharedPtr& node)
: node_(node)
{
    controller_switcher_ = node_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller", 10);
    controller_list_requester_ = node_->create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers", 10);

    auto controller_names = AcquireParam<std::vector<std::string>>("/move_group", "moveit_simple_controller_manager.controller_names");
    RCLCPP_INFO(node_->get_logger(), "Controllers' name acquired form moveit");
    for (const auto & name : controller_names) {
        RCLCPP_INFO(node_->get_logger(), "Moveit claimed controller: %s", name.c_str());
        MOVE_GROUP_CONTROLLER_NAMES.push_back(name);
    }

    switchController();
}

void ControllerSwitcher::switchToControlPad()
{
    switchController({CONTROL_PAD_CONTROLLER_NAME});
}

void ControllerSwitcher::switchToTaskExecutor()
{
    KinematicsPlugin::instance().moveJointPositionAbsolutely(RobotHandle::instance().getCurrentJointPosition());
    switchController({MOVE_GROUP_CONTROLLER_NAMES});
}

void ControllerSwitcher::switchController(std::vector<std::string> activates)
{
    std::vector<std::string> deactivates = listActiveController();
    std::unordered_set<std::string> deactivateSet(deactivates.begin(), deactivates.end());
    std::unordered_set<std::string> common;
    for (const auto& a : activates) {
        if (deactivateSet.count(a)) {
            common.insert(a);
        }
    }
    deactivates.erase(std::remove_if(deactivates.begin(), deactivates.end(),
                           [&](const auto& val) { return common.count(val); }), deactivates.end());
    activates.erase(std::remove_if(activates.begin(), activates.end(),
                           [&](const auto& val) { return common.count(val); }), activates.end());

    if (activates.empty() && deactivates.empty()) {
        // RCLCPP_INFO(node_->get_logger(), "No controller to switch, skipping.");
        return;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController_Request>();
    request->strictness = controller_manager_msgs::srv::SwitchController_Request::STRICT;
    for_each(deactivates.begin(), deactivates.end(), [&](const auto& s) {request->deactivate_controllers.push_back(s);});
    for_each(activates.begin(), activates.end(), [&](const auto& s) {request->activate_controllers.push_back(s);});
    auto response = controller_switcher_->async_send_request(request, [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture result){
        switch_ready_ = true;
        if (result.valid()) {
            switch_success_ = true;
        } else {
            switch_success_ = false;
        }
    });
    while (!switch_ready_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    switch_ready_ = false;
    if(switch_success_) {

    }
    else {
        throw(std::runtime_error("Failed to switch controller!"));
    }
}

std::vector<std::string> ControllerSwitcher::listActiveController()
{
    static const size_t retry_times = 5;
    size_t current_retry_times = 0;
    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers_Request>();
    std::vector<std::string> list;
    while(current_retry_times < retry_times) {
        auto response = controller_list_requester_->async_send_request(request, [&, this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture result){
            list_ready_ = true;
            if (result.valid()) {
                list_success_ = true;
            } else{
                list_success_ = false;
            }
        });
        if (!list_ready_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            ++current_retry_times;
        }
        else {
            if (list_success_)
            {
                auto controller_list = response.get()->controller;
                for(const auto& c : controller_list) {
                    if(c.name.find("controller") != std::string::npos && c.state == "active" ) {
                        list.push_back(c.name);
                    }
                }
            }
            break;
        }
    }
    if(!list_ready_ || !list_success_) {
        throw(std::runtime_error("Failed to list controllers"));
    }
    list_ready_ = false;

    return list;
}
