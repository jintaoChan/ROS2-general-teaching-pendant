#pragma once
#include <optional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <kdl/frames.hpp>
#include <rclcpp/time.hpp>
#include "robot_ports.h"

class IKinematicsSolver {
public:
    virtual ~IKinematicsSolver() = default;

    // Prepare internal solver context for a subsequent cartesian jog step.
    virtual void prepareCartesianJog(const geometry_msgs::msg::Twist& twist_msg) = 0;
    // Compute one cartesian jog target in joint space at the given time.
    virtual std::optional<JointsPosition> computeCartesianJogCommand(const rclcpp::Time& current_time) = 0;
    // Solve IK for an absolute pose without issuing any motion command.
    virtual std::optional<JointsPosition> solvePoseIK(const KDL::Frame& pose) = 0;

    virtual geometry_msgs::msg::PoseStamped getCurrentCartesianPose() = 0;
    virtual void selectCoordinateSystem(const ControlCoordinateSystemType& coord_sys) = 0;
    virtual void refreshCoordinateSystem() = 0;
    virtual KDL::Frame tcpCalibration(const MovePointInfo& points) = 0;
};
