#pragma once

#include <functional>
#include <mutex>
#include <unordered_map>

#include <control_msgs/msg/dynamic_interface_group_values.hpp>
#include <rclcpp/logger.hpp>

#include "robot_handle.h"

class RobotModelStore;

class MotorStatusStore {
public:
    JointsStatus status() const;
    JointsMode mode() const;

    void processStatusMessage(const control_msgs::msg::DynamicInterfaceGroupValues& msg,
                              const RobotModelStore& model,
                              rclcpp::Logger logger);

    size_t registerStatusCallback(MotorStatusCallback cb);
    void unregisterStatusCallback(size_t callback_id);

private:
    JointsMode joint_mode_;
    JointsStatus joint_status_;
    mutable std::mutex state_mutex_;
    std::unordered_map<size_t, MotorStatusCallback> status_callbacks_;
    std::mutex callbacks_mutex_;
    size_t callback_next_id_ = 1;
};
