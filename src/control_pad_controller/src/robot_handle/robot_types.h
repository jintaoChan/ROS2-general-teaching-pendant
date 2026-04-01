#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include <kdl/frames.hpp>

namespace urdf {
class Model;
class ModelInterface;
class Joint;
}

enum class DriverState
{
    STATE_UNDEFINED = 0,
    STATE_START = 1,
    STATE_NOT_READY_TO_SWITCH_ON,
    STATE_SWITCH_ON_DISABLED,
    STATE_READY_TO_SWITCH_ON,
    STATE_SWITCH_ON,
    STATE_OPERATION_ENABLED,
    STATE_QUICK_STOP_ACTIVE,
    STATE_FAULT_REACTION_ACTIVE,
    STATE_FAULT
};

enum class DriverMode : int8_t {
    CSP = 8,
    CST = 10,
};

enum class ControlCoordinateSystemType {
    Base = 0,
    Tool,
    EndEffector
};

struct Joint {
    std::shared_ptr<const urdf::Joint> joint_info;
    double joint_value = 0.0;
};

using JointsPosition = std::unordered_map<std::string, Joint>;
using JointsVelocity = std::unordered_map<std::string, Joint>;
using JointsAcceleration = std::unordered_map<std::string, Joint>;
using JointsTorque = std::unordered_map<std::string, Joint>;
using JointsMode = std::unordered_map<std::string, int8_t>;
using JointsStatus = std::unordered_map<std::string, DriverState>;
using IOValue = std::optional<bool>;
using IOStatus = std::vector<std::pair<std::string, std::vector<std::pair<std::string, IOValue>>>>; //<module_name, <interface_name, val>>
using ToolInfo = std::unordered_map<std::string, KDL::Frame>;

enum class MoveTypeEnum : char {
    POSE = 0,
    JOINT
};

struct TargetPointInfo {
    MoveTypeEnum MoveType;
    JointsPosition JointValues;
    KDL::Frame Pose;
    double VelocityRatio{0.1};
};
using MovePointInfo  = std::unordered_map<std::string, TargetPointInfo>;

using MotorStatusCallback = std::function<void(JointsStatus)>;
using IOStatusCallback = std::function<void(IOStatus)>;
