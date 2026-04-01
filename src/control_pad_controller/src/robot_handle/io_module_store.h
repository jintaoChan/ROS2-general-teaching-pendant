#pragma once

#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <control_msgs/msg/dynamic_interface_group_values.hpp>
#include <rclcpp/logger.hpp>

#include "robot_handle.h"

class IoModuleStore {
public:
    void initGroups(const std::vector<std::string>& input_groups,
                    const std::vector<std::string>& output_groups);

    void setCommandInterfaces(const std::unordered_map<std::string, std::vector<std::string>>& cmd);
    void setMonitorableInterfaces(const std::unordered_map<std::string, std::unordered_set<std::string>>& mon);

    const std::vector<std::string>& inputGroupsName() const;
    const std::vector<std::string>& outputGroupsName() const;
    const std::vector<std::string>& interfacesName(const std::string& module_name) const;
    bool isMonitorable(const std::string& module_name, const std::string& interface_name) const;

    void processStatusMessage(const control_msgs::msg::DynamicInterfaceGroupValues& msg,
                              rclcpp::Logger logger);

    size_t registerStatusCallback(IOStatusCallback cb);
    void unregisterStatusCallback(size_t callback_id);

private:
    std::vector<std::string> input_groups_name_;
    std::vector<std::string> output_groups_name_;
    std::unordered_map<std::string, std::vector<std::string>> group_command_interfaces_;
    std::unordered_map<std::string, std::unordered_set<std::string>> group_monitorable_state_interfaces_;
    IOStatus status_;
    std::unordered_map<size_t, IOStatusCallback> status_callbacks_;
    std::mutex callbacks_mutex_;
    size_t callback_next_id_ = 1;
};
