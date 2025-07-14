#pragma once

#include "robot_description.h"
#include "singleton.hpp"

class MoveExecutor : public Singleton<MoveExecutor> {
    friend class Singleton<MoveExecutor>;
public:
    MoveExecutor(const rclcpp::Node::SharedPtr& node);
public:
    void ExecuteTask(const MoveTasks& task);
    void StopMoving();
    void pubMoveCommand(sensor_msgs::msg::JointState msg);

private:
    rclcpp::Node::SharedPtr m_Node;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_MoveCommandSender;

};
