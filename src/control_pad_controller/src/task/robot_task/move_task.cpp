#include "move_task.h"
#include "kinematics_plugin.h"

void MoveTask::execute()
{
    try{
        switch(target_.MoveType) {
        case(MoveTypeEnum::JOINT): {
            KinematicsPlugin::instance().moveJointPositionAbsolutely(target_.JointValues,target_.VelocityRatio);
            break;
        }
        case(MoveTypeEnum::POSE): {
            KinematicsPlugin::instance().moveToPose(target_.Pose, target_.VelocityRatio);
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
    return !KinematicsPlugin::instance().isRunning();
}

void MoveTask::stop()
{
    return KinematicsPlugin::instance().stop();
}
