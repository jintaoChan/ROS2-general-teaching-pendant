#ifndef MODE_CHANGER_H
#define MODE_CHANGER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include "singleton.hpp"

class ModeChanger : public Singleton<ModeChanger>
{

    friend class Singleton<ModeChanger>;
public:
    ModeChanger(const rclcpp::Node::SharedPtr& node);

public:
    void changeToVelocityMode();
    void changeToPositionMode();

private:
    rclcpp::Node::SharedPtr m_Node;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr m_ModeChangerPublisher;

};

#endif // MODE_CHANGER_H
