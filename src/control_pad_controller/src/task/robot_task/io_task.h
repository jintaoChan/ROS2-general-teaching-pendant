#pragma once

#include "robot_task.h"
#include "robot_handle.h"
#include <string>
#include <functional>
#include <optional>
#include <atomic>
#include <memory>

class IOTask : public RobotTask
{
public:
    IOTask(const std::string& module_name, const std::string& interface_name, bool target_state) :
        RobotTask(),
        module_name_(module_name),
        interface_name_(interface_name),
        target_state_(target_state),
        current_state_(std::nullopt),
        is_executing_(false),
        callback_id_(std::nullopt),
        alive_(std::make_shared<std::atomic<bool>>(true)) {}

    ~IOTask() override
    {
        *alive_ = false;
        stop();
    }

    virtual void execute() override final;
    virtual bool isFinished() override final;
    virtual void stop() override final;

private:
    std::string module_name_;
    std::string interface_name_;
    bool target_state_;
    std::optional<bool> current_state_;
    bool is_executing_;
    std::optional<size_t> callback_id_;
    // Sentinel shared with the RobotHandle callback: set to false on destruction
    // so the callback becomes a no-op after IOTask is gone.
    std::shared_ptr<std::atomic<bool>> alive_;

    void onIOStatusUpdate(const IOStatus& io_status);
};

