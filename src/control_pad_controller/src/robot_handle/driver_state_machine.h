#pragma once

#include <cstdint>

#include "robot_handle.h"

class DriverStateMachine {
public:
    static DriverState fromStatusWord(uint16_t status_word);
    static uint16_t transitionControlWord(DriverState state, uint16_t control_word);
};
