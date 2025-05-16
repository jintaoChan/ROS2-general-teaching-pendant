#pragma once

#include "robot_description.hpp"

// extern std::atomic<bool> KeepMoving;
void ExecuteTask(const MoveTasks& task);
void StopMoving();
