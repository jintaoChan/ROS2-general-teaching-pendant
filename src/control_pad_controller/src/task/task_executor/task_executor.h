#pragma once

#include <deque>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include "robot_task.h"
#include "singleton.hpp"

class TaskExecutor : public Singleton<TaskExecutor> {
    friend class Singleton<TaskExecutor>;

private:
    std::deque<std::unique_ptr<RobotTask>> task_queue_;
    std::unique_ptr<RobotTask> current_task_;

    std::thread worker_thread_;
    std::mutex queue_mutex_;
    std::atomic<bool> running_;
    std::atomic<bool> paused_;

    void run() {
        while (running_) {
            if (paused_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (!current_task_ && !task_queue_.empty()) {
                current_task_ = std::move(task_queue_.front());
                task_queue_.pop_front();
                current_task_->execute();
            }

            if (current_task_) {
                if (current_task_->isFinished()) {
                    current_task_.reset();
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

public:
    TaskExecutor() : current_task_(nullptr), running_(false), paused_(false){}

    ~TaskExecutor() { stop(); }

    void start() {
        if (!running_) {
            running_ = true;
            worker_thread_ = std::thread(&TaskExecutor::run, this);
        }
    }

    void stop() {
        running_ = false;
        if (worker_thread_.joinable()) worker_thread_.join();
        clearTasks();
    }

    void addTask(std::unique_ptr<RobotTask> task) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        task_queue_.push_back(std::move(task));
    }

    void clearTasks() {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        task_queue_.clear();
        current_task_.reset();
    }

    void setPause(bool p) { paused_ = p; }
};
