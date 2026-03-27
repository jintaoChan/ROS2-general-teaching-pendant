#pragma once

#include "robot_ports.h"

// Lightweight aggregate for UI dependencies, to avoid threading multiple
// interface pointers through each widget constructor.
struct AppPorts {
    IRobotStateProvider* state{nullptr};
    IRobotCommandPort* command{nullptr};
    IRobotEvents* events{nullptr};
};
