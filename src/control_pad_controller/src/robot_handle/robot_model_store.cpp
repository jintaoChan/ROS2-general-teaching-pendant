#include "robot_model_store.h"

urdf::Model& RobotModelStore::model() {
    return model_;
}

const urdf::Model& RobotModelStore::model() const {
    return model_;
}

KDL::Tree& RobotModelStore::kdlTree() {
    return kdl_tree_;
}

const KDL::Tree& RobotModelStore::kdlTree() const {
    return kdl_tree_;
}

KDL::Chain& RobotModelStore::kdlChain() {
    return kdl_chain_;
}

const KDL::Chain& RobotModelStore::kdlChain() const {
    return kdl_chain_;
}

std::vector<std::string>& RobotModelStore::jointNames() {
    return joint_names_;
}

const std::vector<std::string>& RobotModelStore::jointNames() const {
    return joint_names_;
}

JointsPosition& RobotModelStore::currentJointPosition() {
    return current_joint_position_;
}

const JointsPosition& RobotModelStore::currentJointPosition() const {
    return current_joint_position_;
}

JointsVelocity& RobotModelStore::currentJointVelocity() {
    return current_joint_velocity_;
}

const JointsVelocity& RobotModelStore::currentJointVelocity() const {
    return current_joint_velocity_;
}

JointsTorque& RobotModelStore::currentJointTorque() {
    return current_joint_torque_;
}

const JointsTorque& RobotModelStore::currentJointTorque() const {
    return current_joint_torque_;
}

std::string& RobotModelStore::robotArmBaseLinkName() {
    return robot_arm_base_link_name_;
}

const std::string& RobotModelStore::robotArmBaseLinkName() const {
    return robot_arm_base_link_name_;
}

std::string& RobotModelStore::robotArmEndLinkName() {
    return robot_arm_end_link_name_;
}

const std::string& RobotModelStore::robotArmEndLinkName() const {
    return robot_arm_end_link_name_;
}
