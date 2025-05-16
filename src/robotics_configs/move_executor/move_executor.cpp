#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include "move_executor.hpp"
#include "controller_switcher.h"
#include "mode_changer.h"

std::atomic<bool> KeepMoving = true;

void ExecuteTask(const MoveTasks &task)
{
    auto executePoint = [](const MovePointInfo& moveGroupInfo) {
        auto& robotDes = RobotDescription::instance();
        for(const auto& groupInfo : moveGroupInfo) {
            auto& moveGroupIF = robotDes.getMoveGroupInterfaces(groupInfo.first);
            moveGroupIF->setJointValueTarget(groupInfo.second.Values);
            moveGroupIF->setMaxVelocityScalingFactor(1);
            moveGroupIF->setMaxAccelerationScalingFactor(1);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (moveGroupIF->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success && KeepMoving) {
                moveGroupIF->execute(plan);
            }
        }
    };
    ControllerSwitcher::instance().switchToTaskExecutor();
    ModeChanger::instance().changeToPositionMode();
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

void StopMoving()
{
    auto& robotDes = RobotDescription::instance();
    KeepMoving = false;
    auto jointGroups = robotDes.getJointGroupsNames();
    for(const auto& group : jointGroups) {
        auto& moveGroupIF = robotDes.getMoveGroupInterfaces(group);
        moveGroupIF->stop();
    }
}
