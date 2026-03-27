#include "move_task.h"
#include "motion_plugin.h"

void MoveTask::execute()
{
    try{
        switch(target_.MoveType) {
        case(MoveTypeEnum::JOINT): {
            MotionPlugin::instance().moveJointPositionAbsolutely(target_.JointValues,target_.VelocityRatio);
            break;
        }
        case(MoveTypeEnum::POSE): {
            MotionPlugin::instance().moveToPose(target_.Pose, target_.VelocityRatio);
            break;
        }
        }
    }
    catch(std::exception& e){
        std::cout << e.what() << std::endl;
        throw(std::runtime_error("Run into error! Please check logs!"));
    }
}

bool MoveTask::isFinished()
{
    return !MotionPlugin::instance().isRunning();
}

void MoveTask::stop()
{
    return MotionPlugin::instance().stop();
}
