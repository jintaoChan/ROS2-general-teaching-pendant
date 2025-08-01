#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include "move_executor.h"
#include "controller_switcher.h"

std::atomic<bool> KeepMoving;

void MoveExecutor::ExecuteTask(const MoveTasks &task, double velocity_scaling_factor)
{
    auto executePoint = [velocity_scaling_factor](const MovePointInfo& moveGroupInfo) {
        auto& robotDes = RobotHandle::instance();
        for(const auto& groupInfo : moveGroupInfo) {
            auto& moveGroupIF = robotDes.getMoveGroupInterfaces(groupInfo.first);
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
        // const auto& pointName = p.PointName;
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
    auto& robotDes = RobotHandle::instance();
    KeepMoving = false;
    auto jointGroups = robotDes.getJointGroupsNames();
    for(const auto& group : jointGroups) {
        auto& moveGroupIF = robotDes.getMoveGroupInterfaces(group);
        moveGroupIF->stop();
    }
}
