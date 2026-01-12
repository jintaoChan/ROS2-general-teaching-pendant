#pragma once

#include <deque>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include "robot_task.h"
#include "singleton.hpp"

enum class ExecutorState {
    IDLE,
    RUNNING,
    PAUSED
};


class TaskExecutor : public Singleton<TaskExecutor> {
    friend class Singleton<TaskExecutor>;
    using StateCallback = std::function<void(ExecutorState)>;

public:
    TaskExecutor() : current_task_(nullptr), running_(false), paused_(false){
        thread_lifecycle_running_ = true;
        state_ = ExecutorState::IDLE;

        worker_thread_ = std::thread(&TaskExecutor::run, this);
    }

    ~TaskExecutor() {
        running_ = false;
        if (worker_thread_.joinable()) {
            if (std::this_thread::get_id() != worker_thread_.get_id()) {
                worker_thread_.join();
            } else {
                worker_thread_.detach();
            }
        }
    }

    void setStateCallback(StateCallback cb) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        state_callback_ = cb;
    }

    bool start() {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (task_queue_.empty() || state_ != ExecutorState::IDLE) return false;

        updateState(ExecutorState::RUNNING);
        return true;
    }

    void stop() {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (current_task_) {
            current_task_->stop();
            current_task_.reset();
        }
        task_queue_.clear();
        updateState(ExecutorState::IDLE);
        paused_ = false;
    }

    bool addTask(std::unique_ptr<RobotTask> task) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (state_ != ExecutorState::IDLE) {
            return false;
        }
        task_queue_.push_back(std::move(task));
        return true;
    }

    void clearTasks() {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        task_queue_.clear();
        current_task_.reset();
    }

    void setPause(bool p) { paused_ = p; }

private:
    std::deque<std::unique_ptr<RobotTask>> task_queue_;
    std::unique_ptr<RobotTask> current_task_;
    std::atomic<ExecutorState> state_{ExecutorState::IDLE};
    std::atomic<bool> thread_lifecycle_running_;
    std::thread worker_thread_;
    std::mutex queue_mutex_;
    std::atomic<bool> running_;
    std::atomic<bool> paused_;
    StateCallback state_callback_ = nullptr;

    void updateState(ExecutorState new_state) {
        if (state_ == new_state) return;
        state_ = new_state;
        if (state_callback_) {
            state_callback_(new_state);
        }
    }

    void run() {
        while (thread_lifecycle_running_) {
            if (state_ == ExecutorState::RUNNING) {
                std::lock_guard<std::mutex> lock(queue_mutex_);

                if (!current_task_ && !task_queue_.empty()) {
                    current_task_ = std::move(task_queue_.front());
                    task_queue_.pop_front();
                    current_task_->execute();
                } else if (current_task_) {
                    if (current_task_->isFinished()) {
                        current_task_.reset();
                        if (task_queue_.empty()) {
                            updateState(ExecutorState::IDLE);
                        }
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

};
