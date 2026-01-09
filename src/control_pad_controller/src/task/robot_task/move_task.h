#pragma once

#include "robot_task.h"
#include "robot_handle.h"


class MoveTask : public RobotTask
{
public:
    MoveTask(const TargetPointInfo& p) :
        RobotTask(),
        target_(p) {}

    virtual void execute() override final;
    virtual bool isFinished() override final;
    virtual void stop() override final;

private:
    TargetPointInfo target_;
};

