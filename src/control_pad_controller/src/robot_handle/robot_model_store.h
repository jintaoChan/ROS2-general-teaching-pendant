#pragma once

#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>

#include "robot_handle.h"

class RobotModelStore {
public:
    urdf::Model& model();
    const urdf::Model& model() const;

    KDL::Tree& kdlTree();
    const KDL::Tree& kdlTree() const;

    KDL::Chain& kdlChain();
    const KDL::Chain& kdlChain() const;

    std::vector<std::string>& jointNames();
    const std::vector<std::string>& jointNames() const;

    JointsPosition& currentJointPosition();
    const JointsPosition& currentJointPosition() const;

    JointsVelocity& currentJointVelocity();
    const JointsVelocity& currentJointVelocity() const;

    JointsTorque& currentJointTorque();
    const JointsTorque& currentJointTorque() const;

    std::string& robotArmBaseLinkName();
    const std::string& robotArmBaseLinkName() const;

    std::string& robotArmEndLinkName();
    const std::string& robotArmEndLinkName() const;

private:
    urdf::Model model_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;

    std::vector<std::string> joint_names_;
    JointsPosition current_joint_position_;
    JointsVelocity current_joint_velocity_;
    JointsTorque current_joint_torque_;

    std::string robot_arm_base_link_name_;
    std::string robot_arm_end_link_name_;
};
