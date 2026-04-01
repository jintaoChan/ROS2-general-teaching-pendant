#include "io_module_store.h"

#include <algorithm>
#include <cmath>

#include <rclcpp/logging.hpp>

void IoModuleStore::initGroups(const std::vector<std::string>& input_groups,
                                const std::vector<std::string>& output_groups)
{
    input_groups_name_ = input_groups;
    output_groups_name_ = output_groups;
    group_command_interfaces_.clear();
    group_monitorable_state_interfaces_.clear();
    for (const auto& g : input_groups_name_) {
        group_command_interfaces_[g] = {};
        group_monitorable_state_interfaces_[g] = {};
    }
    for (const auto& g : output_groups_name_) {
        group_command_interfaces_[g] = {};
        group_monitorable_state_interfaces_[g] = {};
    }
}

void IoModuleStore::setCommandInterfaces(const std::unordered_map<std::string, std::vector<std::string>>& cmd)
{
    group_command_interfaces_ = cmd;
}

void IoModuleStore::setMonitorableInterfaces(const std::unordered_map<std::string, std::unordered_set<std::string>>& mon)
{
    group_monitorable_state_interfaces_ = mon;
}

const std::vector<std::string>& IoModuleStore::inputGroupsName() const
{
    return input_groups_name_;
}

const std::vector<std::string>& IoModuleStore::outputGroupsName() const
{
    return output_groups_name_;
}

const std::vector<std::string>& IoModuleStore::interfacesName(const std::string& module_name) const
{
    static const std::vector<std::string> kEmpty;
    auto it = group_command_interfaces_.find(module_name);
    if (it == group_command_interfaces_.end()) {
        return kEmpty;
    }
    return it->second;
}

bool IoModuleStore::isMonitorable(const std::string& module_name, const std::string& interface_name) const
{
    auto it = group_monitorable_state_interfaces_.find(module_name);
    if (it == group_monitorable_state_interfaces_.end()) {
        return false;
    }
    return it->second.find(interface_name) != it->second.end();
}

void IoModuleStore::processStatusMessage(const control_msgs::msg::DynamicInterfaceGroupValues& msg,
                                          rclcpp::Logger logger)
{
    constexpr double kBoolTolerance = 1e-6;

    status_.clear();
    for (size_t i = 0; i < msg.interface_groups.size(); ++i) {
        const auto& vals = msg.interface_values[i];
        const auto& module_name = msg.interface_groups[i];
        const bool known =
            std::find(input_groups_name_.begin(), input_groups_name_.end(), module_name) != input_groups_name_.end() ||
            std::find(output_groups_name_.begin(), output_groups_name_.end(), module_name) != output_groups_name_.end();
        if (!known) {
            RCLCPP_WARN(logger, "Detected IO module have not been registered");
            continue;
        }
        status_.push_back({module_name, {}});
        for (size_t j = 0; j < vals.interface_names.size(); ++j) {
            const double raw = vals.values[j];
            IOValue io_value = std::nullopt;
            if (std::isfinite(raw)) {
                if (std::abs(raw) <= kBoolTolerance) {
                    io_value = false;
                } else if (std::abs(raw - 1.0) <= kBoolTolerance) {
                    io_value = true;
                }
            }
            status_.back().second.push_back({vals.interface_names[j], io_value});
        }
    }

    std::vector<IOStatusCallback> callbacks;
    {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        callbacks.reserve(status_callbacks_.size());
        for (const auto& [_, cb] : status_callbacks_) {
            callbacks.push_back(cb);
        }
    }
    for (const auto& cb : callbacks) {
        cb(status_);
    }
}

size_t IoModuleStore::registerStatusCallback(IOStatusCallback cb)
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    const size_t id = callback_next_id_++;
    status_callbacks_[id] = std::move(cb);
    return id;
}

void IoModuleStore::unregisterStatusCallback(size_t callback_id)
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    status_callbacks_.erase(callback_id);
}
