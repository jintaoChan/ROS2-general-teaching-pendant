#pragma once

#include <string>

#include <kdl/frames.hpp>

#include "robot_handle.h"

class ToolFrameStore {
public:
    const ToolInfo& getToolInfo() const;
    void deleteToolFrame(const std::string& tool_name);
    void addToolFrame(const std::string& tool_name, const KDL::Frame& frame);
    void setCurrentToolFrame(const std::string& tool_name);

    const bool& isToolFrameSet() const;
    const std::string& getCurrentToolFrame() const;

    void initializeFromLoadedFrames(const ToolInfo& loaded_frames);

private:
    ToolInfo robot_arm_tool_info_;
    bool tool_frame_set_{false};
    std::string current_tool_frame_;
};
