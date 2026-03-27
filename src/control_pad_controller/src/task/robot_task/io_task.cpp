#include "io_task.h"

IRobotCommandPort* IOTask::command_port_ = nullptr;
IRobotEvents* IOTask::event_port_ = nullptr;

void IOTask::configurePorts(IRobotCommandPort* command_port, IRobotEvents* event_port)
{
    command_port_ = command_port;
    event_port_ = event_port;
}

void IOTask::execute()
{
    if (!is_executing_) {
        try {
            if (command_port_ == nullptr || event_port_ == nullptr) {
                throw(std::runtime_error("IOTask ports are not configured"));
            }

            // Capture a weak_ptr to the alive sentinel so the callback becomes
            // a no-op automatically once this IOTask instance is destroyed.
            std::weak_ptr<std::atomic<bool>> weak_alive = alive_;
            callback_id_ = event_port_->registerIOStatusCallback(
                [this, weak_alive](const IOStatus& io_status) {
                    auto alive = weak_alive.lock();
                    if (!alive || !(*alive)) return;
                    onIOStatusUpdate(io_status);
                }
            );

            // Set the IO state to target value
            command_port_->setIOState(module_name_, interface_name_, target_state_);

            is_executing_ = true;
        }
        catch (std::exception& e) {
            std::cout << e.what() << std::endl;
            throw(std::runtime_error("Failed to execute IO task! Please check logs!"));
        }
    }
}

bool IOTask::isFinished()
{
    // Task is finished when current state matches target state
    return is_executing_ && current_state_.has_value() && current_state_.value() == target_state_;
}

void IOTask::stop()
{
    if (callback_id_.has_value()) {
        if (event_port_ != nullptr) {
            event_port_->unregisterIOStatusCallback(callback_id_.value());
        }
        callback_id_ = std::nullopt;
    }

    is_executing_ = false;
    current_state_ = std::nullopt;
}

void IOTask::onIOStatusUpdate(const IOStatus& io_status)
{
    // Search for the target module and interface in the IO status
    for (const auto& module : io_status) {
        if (module.first == module_name_) {
            for (const auto& interface : module.second) {
                if (interface.first == interface_name_) {
                    current_state_ = interface.second;
                    return;
                }
            }
        }
    }
}

