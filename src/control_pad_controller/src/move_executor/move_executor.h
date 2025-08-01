#pragma once

#include "robot_handle.h"
#include "singleton.hpp"

namespace MoveExecutor {

    void ExecuteTask(const MoveTasks& task, double velocity_scaling_factor);
    void StopMoving();
    
}
