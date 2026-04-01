#pragma once

#include <mutex>
#include <magic_enum/magic_enum.hpp>

enum class MotionState {
    Idle,
    CartesianJog,
    JointJog,
    TrajectoryExec,
    Dragging,
    Error
};

// Thread-safe motion mode state machine.
// Only one motion mode is active at a time.
// Transitions: Idle -> any active state; any state -> Idle via forceIdle().
// Re-entering the same state is allowed (idempotent).
class MotionStateMachine {
public:
    bool tryTransition(MotionState target) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ == target) return true;
        if (state_ != MotionState::Idle) return false;
        state_ = target;
        return true;
    }

    void forceIdle() {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = MotionState::Idle;
    }

    MotionState currentState() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_;
    }

private:
    mutable std::mutex mutex_;
    MotionState state_{MotionState::Idle};
};
