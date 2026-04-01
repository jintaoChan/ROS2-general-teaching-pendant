#include "driver_state_machine.h"

DriverState DriverStateMachine::fromStatusWord(uint16_t status_word)
{
    if ((status_word & 0b01001111) == 0b00000000) {
        return DriverState::STATE_NOT_READY_TO_SWITCH_ON;
    }
    if ((status_word & 0b01001111) == 0b01000000) {
        return DriverState::STATE_SWITCH_ON_DISABLED;
    }
    if ((status_word & 0b01101111) == 0b00100001) {
        return DriverState::STATE_READY_TO_SWITCH_ON;
    }
    if ((status_word & 0b01101111) == 0b00100011) {
        return DriverState::STATE_SWITCH_ON;
    }
    if ((status_word & 0b01101111) == 0b00100111) {
        return DriverState::STATE_OPERATION_ENABLED;
    }
    if ((status_word & 0b01101111) == 0b00000111) {
        return DriverState::STATE_QUICK_STOP_ACTIVE;
    }
    if ((status_word & 0b01001111) == 0b00001111) {
        return DriverState::STATE_FAULT_REACTION_ACTIVE;
    }
    if ((status_word & 0b01001111) == 0b00001000) {
        return DriverState::STATE_FAULT;
    }
    return DriverState::STATE_UNDEFINED;
}

uint16_t DriverStateMachine::transitionControlWord(DriverState state, uint16_t control_word)
{
    switch (state) {
    case DriverState::STATE_START:
        return control_word;
    case DriverState::STATE_NOT_READY_TO_SWITCH_ON:
        return control_word;
    case DriverState::STATE_SWITCH_ON_DISABLED:
        return (control_word & 0b01111110) | 0b00000110;
    case DriverState::STATE_READY_TO_SWITCH_ON:
        return (control_word & 0b01110111) | 0b00000111;
    case DriverState::STATE_SWITCH_ON:
        return (control_word & 0b01111111) | 0b00001111;
    case DriverState::STATE_OPERATION_ENABLED:
        return control_word | 0b00011111;
    case DriverState::STATE_QUICK_STOP_ACTIVE:
        return (control_word & 0b01111111) | 0b00001111;
    case DriverState::STATE_FAULT_REACTION_ACTIVE:
        return control_word;
    case DriverState::STATE_FAULT:
        return 0;
    default:
        break;
    }
    return control_word;
}
