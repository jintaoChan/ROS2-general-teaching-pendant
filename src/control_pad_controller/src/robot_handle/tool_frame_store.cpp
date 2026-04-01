#include "tool_frame_store.h"

const ToolInfo& ToolFrameStore::getToolInfo() const {
    return robot_arm_tool_info_;
}

void ToolFrameStore::deleteToolFrame(const std::string& tool_name) {
    robot_arm_tool_info_.erase(tool_name);
}

void ToolFrameStore::addToolFrame(const std::string& tool_name, const KDL::Frame& frame) {
    robot_arm_tool_info_[tool_name] = frame;
}

void ToolFrameStore::setCurrentToolFrame(const std::string& tool_name) {
    current_tool_frame_ = tool_name;
}

const bool& ToolFrameStore::isToolFrameSet() const {
    return tool_frame_set_;
}

const std::string& ToolFrameStore::getCurrentToolFrame() const {
    return current_tool_frame_;
}

void ToolFrameStore::initializeFromLoadedFrames(const ToolInfo& loaded_frames) {
    robot_arm_tool_info_ = loaded_frames;
    if (!robot_arm_tool_info_.empty()) {
        tool_frame_set_ = true;
        current_tool_frame_ = robot_arm_tool_info_.begin()->first;
    }
}
