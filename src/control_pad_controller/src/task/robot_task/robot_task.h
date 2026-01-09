#pragma once

class RobotTask {
public:
    virtual ~RobotTask() {}
    virtual void execute() = 0;
    virtual bool isFinished() = 0;
    virtual void stop() = 0;
};