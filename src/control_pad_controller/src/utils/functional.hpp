#pragma once

#include <rclcpp/rclcpp.hpp>

template<typename T>
T AcquireParam(const std::string& node_name, const std::string& param_name) {
    auto node = rclcpp::Node::make_shared("param_acquire");
    T param;
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, node_name);

    while (!param_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(node->get_logger(), "waiting for %s param service...", node_name.c_str());
    }

    if (param_client->has_parameter(param_name)) {
        param = param_client->get_parameter(param_name, T{});
    } else {
        RCLCPP_WARN(node->get_logger(), "Failed to find param: %s", param_name.c_str());
    }
    return param;
}