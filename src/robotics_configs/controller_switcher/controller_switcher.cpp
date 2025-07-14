#include "controller_switcher.h"

ControllerSwitcher::ControllerSwitcher(const rclcpp::Node::SharedPtr& node)
    : m_Node(node)
{
    m_ControllerSwitcher = m_Node->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller", 10);
    m_ControllerListRequester = m_Node->create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers", 10);


    // deactivate moveit controller

    switchController();
}

void ControllerSwitcher::switchToControlPad() const
{
    switchController({CONTROL_PAD_CONTROLLER_NAME});
}

void ControllerSwitcher::switchToTaskExecutor() const
{
    switchController({TASK_EXECUTOR_CONTROLLER_NAME});
}

void ControllerSwitcher::switchController(std::vector<std::string> activates) const
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
        // RCLCPP_INFO(m_Node->get_logger(), "No controller to switch, skipping.");
        return;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController_Request>();
    request->strictness = controller_manager_msgs::srv::SwitchController_Request::STRICT;
    for_each(deactivates.begin(), deactivates.end(), [&](const auto& s) {request->deactivate_controllers.push_back(s);});
    for_each(activates.begin(), activates.end(), [&](const auto& s) {request->activate_controllers.push_back(s);});
    auto response = m_ControllerSwitcher->async_send_request(request);
    if (rclcpp::spin_until_future_complete(m_Node, response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        // RCLCPP_INFO(m_Node->get_logger(), "Succeed to switch off controller");
    } else {
        RCLCPP_ERROR(m_Node->get_logger(), "Failed to call service!");
    }
}

std::vector<std::string> ControllerSwitcher::listActiveController() const
{
    std::vector<std::string> list;
    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers_Request>();
    auto response = m_ControllerListRequester->async_send_request(request);
    std::vector<std::string> deactivates, activates;
    if (rclcpp::spin_until_future_complete(m_Node, response) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto controllerList = response.get()->controller;
        for(const auto& c : controllerList) {
            if(c.name.find("controller") != std::string::npos && c.state == "active" ) {
                list.push_back(c.name);
            }
        }

    } else {
        RCLCPP_ERROR(m_Node->get_logger(), "Failed to list controllers!");
    }
    return list;
}
