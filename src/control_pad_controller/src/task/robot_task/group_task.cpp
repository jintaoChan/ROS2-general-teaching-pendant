#include "group_task.h"


void GroupTask::execute()
{
    current_loop_ = 0;
    current_index_ = 0;
    if (repeat_times_ <= 0 || task_list_.empty()) {
        return;
    }

    current_loop_ = 1;
    if (current_index_ < task_list_.size()) {
        task_list_[current_index_]->execute();
    }
}

bool GroupTask::isFinished()
{
    if (repeat_times_ <= 0 || task_list_.empty()) return true;
    if (current_index_ >= task_list_.size()) return true;

    if (task_list_[current_index_]->isFinished()) {
        ++current_index_;

        if (current_index_ >= task_list_.size()) {
            if (current_loop_ < repeat_times_) {
                ++current_loop_;
                current_index_ = 0;
                task_list_[current_index_]->execute();
            } else {
                return true;
            }
        } else {
            task_list_[current_index_]->execute();
        }
    }
    return false;
}

void GroupTask::stop()
{
    if (!task_list_.empty() && current_index_ < task_list_.size()) {
        return task_list_[current_index_]->stop();
    }
}
