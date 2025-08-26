#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include "move_executor.h"
#include "controller_switcher.h"

std::atomic<bool> KeepMoving;

void MoveExecutor::ExecuteTask(const MoveTasks &task, double velocity_scaling_factor)
{
    auto executePoint = [velocity_scaling_factor](const MovePointInfo& moveGroupInfo) {
        auto& robot_des = RobotHandle::instance();
        for(const auto& groupInfo : moveGroupInfo) {
            auto& moveGroupIF = robot_des.getMoveGroupInterfaces(groupInfo.first);
            moveGroupIF->setJointValueTarget(groupInfo.second.Values);
            moveGroupIF->setMaxVelocityScalingFactor(velocity_scaling_factor);
            moveGroupIF->setMaxAccelerationScalingFactor(1);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (moveGroupIF->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success && KeepMoving) {
                moveGroupIF->execute(plan);
            }
        }
    };
    ControllerSwitcher::instance().switchToTaskExecutor();
    KeepMoving = true;
    for (const auto& p : task) {
        // const auto& point_name = p.PointName;
        const auto& pointInfos = p.PointInfos;
        if (pointInfos.index() == 0) {
            auto moveGourps = std::get<0>(pointInfos);
            executePoint(moveGourps);
        }
        else if (pointInfos.index() == 1) {
            auto pointGourps = std::get<1>(pointInfos);
            auto times = pointGourps.Times;
            if(times == -1) {
                while(KeepMoving) {
                    for(const auto& pointGroup : pointGourps.Points) {
                        executePoint(pointGroup);
                    }
                }
            }
            else {
                for(int i = 0; i < times && KeepMoving; ++i) {
                    for(const auto& pointGroup : pointGourps.Points) {
                        executePoint(pointGroup);
                    }
                }
            }
        }
    }
}

void MoveExecutor::StopMoving()
{
    auto& robot_des = RobotHandle::instance();
    KeepMoving = false;
    auto jointGroups = robot_des.getJointGroupsNames();
    for(const auto& group : jointGroups) {
        auto& moveGroupIF = robot_des.getMoveGroupInterfaces(group);
        moveGroupIF->stop();
    }
}
