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
    changeMode(modeMsg);
}

void ModeChanger::changeToPositionMode()
{
    std_msgs::msg::Int8 modeMsg;
    modeMsg.data = 8;
    changeMode(modeMsg);
}

std::shared_ptr<std_msgs::msg::Int8> ModeChanger::getCurrentMode()
{
    std::shared_ptr<std_msgs::msg::Int8> currentMode{nullptr};
    auto sub = m_Node->create_subscription<std_msgs::msg::Int8>(
    "/control_pad_mode_state", 10,
    [&](const std_msgs::msg::Int8::SharedPtr msg) {
        currentMode = msg;
    });

    rclcpp::Rate rate(100);
    rclcpp::Time start = m_Node->now();

    while (rclcpp::ok() && currentMode == nullptr) {
        rclcpp::spin_some(m_Node);
        if ((m_Node->now() - start).seconds() > 2.0) {
            RCLCPP_WARN(m_Node->get_logger(), "Timeout waiting for joint_states");
            break;
        }
        rate.sleep();
    }
    return currentMode;
}

void ModeChanger::changeMode(std_msgs::msg::Int8 mode)
{
    auto currentMode = getCurrentMode();
    if(*currentMode != mode ) {
        m_ModeChangerPublisher->publish(mode);
    }

}
