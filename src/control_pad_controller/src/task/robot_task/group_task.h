#pragma once
#include "robot_task.h"
#include <memory>
#include <vector>
#include <string>


class GroupTask : public RobotTask
{
public:
    GroupTask(const std::string& task_name, int repeat_times) :
        RobotTask(),
        task_name_(task_name),
        repeat_times_(repeat_times) {}

    virtual void execute() override final;
    virtual bool isFinished() override final;
    virtual void stop() override final;

    void addAction(std::unique_ptr<RobotTask> task) {
        task_list_.push_back(std::move(task));
    }
private:
    std::vector<std::unique_ptr<RobotTask>> task_list_;
    std::string task_name_;
    int repeat_times_;
    int current_loop_ = 0;
    size_t current_index_ = 0;
};

