#include "controller_switcher.h"

ControllerSwitcher::ControllerSwitcher(const rclcpp::Node::SharedPtr &node)
:
    node_(node)
{
    switch_controller_client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
    while (!switch_controller_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
    }
}

void ControllerSwitcher::switchToJTC()
{
    switchController({"trajectory_controller"}, {"effort_controller"});
}

void ControllerSwitcher::switchToEffect()
{
    switchController({"effort_controller"}, {"trajectory_controller"});
}

void ControllerSwitcher::switchController(const std::vector<std::string>& activate, const std::vector<std::string>& deactivate) {
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers = activate;
    request->deactivate_controllers = deactivate;
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    auto result_future = switch_controller_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result_future.get();
        if (response->ok) {
            RCLCPP_INFO(node_->get_logger(), "Controller switch successful!");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Controller switch FAILED!");
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service /controller_manager/switch_controller");
    }
    
}
