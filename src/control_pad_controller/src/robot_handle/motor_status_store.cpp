#include "motor_status_store.h"

#include <algorithm>

#include "driver_state_machine.h"
#include "robot_model_store.h"

JointsStatus MotorStatusStore::status() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return joint_status_;
}

JointsMode MotorStatusStore::mode() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return joint_mode_;
}

void MotorStatusStore::processStatusMessage(const control_msgs::msg::DynamicInterfaceGroupValues& msg,
                                             const RobotModelStore& model,
                                             rclcpp::Logger logger)
{
    JointsStatus status_snapshot;
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        for (size_t i = 0; i < msg.interface_groups.size(); ++i) {
            const auto& vals = msg.interface_values[i];
            const auto& joint_name = msg.interface_groups[i];
            if (model.currentJointPosition().find(joint_name) == model.currentJointPosition().end()) {
                RCLCPP_WARN(logger, "Getting a driver status which does not exist in urdf!");
                continue;
            }
            for (size_t j = 0; j < vals.interface_names.size(); ++j) {
                const auto& if_name = vals.interface_names[j];
                if (if_name == "mode") {
                    joint_mode_[joint_name] = (int8_t)vals.values[j];
                } else if (if_name == "status_word") {
                    joint_status_[joint_name] = DriverStateMachine::fromStatusWord(vals.values[j]);
                } else {
                    RCLCPP_WARN(logger, "Getting undefined interface from driver!");
                }
            }
        }
        status_snapshot = joint_status_;
    }

    std::vector<MotorStatusCallback> callbacks;
    {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        callbacks.reserve(status_callbacks_.size());
        for (const auto& [_, cb] : status_callbacks_) {
            callbacks.push_back(cb);
        }
    }
    for (const auto& cb : callbacks) {
        cb(status_snapshot);
    }
}

size_t MotorStatusStore::registerStatusCallback(MotorStatusCallback cb) {
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    const size_t id = callback_next_id_++;
    status_callbacks_[id] = std::move(cb);
    return id;
}

void MotorStatusStore::unregisterStatusCallback(size_t callback_id) {
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    status_callbacks_.erase(callback_id);
}
