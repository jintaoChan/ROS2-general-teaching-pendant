#include "mode_changer.h"

ModeChanger::ModeChanger(const rclcpp::Node::SharedPtr& node)
    : m_Node(node)
{
  m_ModeChangerPublisher = m_Node->create_publisher<std_msgs::msg::Int8>("/control_pad_mode_cmd", 10);
}

void ModeChanger::changeToVelocityMode()
{
    std_msgs::msg::Int8 modeMsg;
    modeMsg.data = 9;
    m_ModeChangerPublisher->publish(modeMsg);

}

void ModeChanger::changeToPositionMode()
{
    std_msgs::msg::Int8 modeMsg;
    modeMsg.data = 8;
    m_ModeChangerPublisher->publish(modeMsg);
}
