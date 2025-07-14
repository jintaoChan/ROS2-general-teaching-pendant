#ifndef CONTROL_PAD_H
#define CONTROL_PAD_H

#include <QThread>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "move_button.h"
#include "cartesian_pad.h"
#include "setting_panel.h"
#include "robot_description.h"

namespace Ui {
class ControlPad;
}

class ControlPad : public QWidget
{
    Q_OBJECT

public:
    explicit ControlPad(QWidget *parent = nullptr);
    ~ControlPad();

public:
    void recvCallback(const sensor_msgs::msg::JointState& msg);
    void run();

public slots:
    void MoveCommander(MoveButtonType type, MoveButtonEvent event, const std::string& jointName);
    void MoveCommander(MoveButtonType type, MoveButtonEvent event, size_t idx);

signals:
    void storeCurrentPointToPointPool(const MovePointInfo& point);

private:
    Ui::ControlPad *ui;
    rclcpp::Node::SharedPtr m_Node;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_JointStateSubscription;
    std::unordered_map<std::string, double> m_CurrentPosition;
    SettingPanel m_SettingPanel;
};

#endif // CONTROL_PAD_H
