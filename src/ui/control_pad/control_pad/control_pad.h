#ifndef CONTROL_PAD_H
#define CONTROL_PAD_H

#include <QThread>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <QTimer>
#include "sensor_msgs/msg/joint_state.hpp"
#include "move_button.h"
#include "cartesian_pad.h"
#include "setting_panel.h"
#include "robot_handle.h"

namespace Ui {
class ControlPad;
}

class ControlPad : public QWidget
{
    Q_OBJECT

public:
    explicit ControlPad(SettingPanel* setting_panel,  QWidget *parent = nullptr);
    ~ControlPad();

public:

private:
    void regularUpdate();

public slots:
    void MoveCommander(MoveButtonType type, MoveButtonEvent event, const std::string& jointName) const ;
    void MoveCommander(MoveButtonType type, MoveButtonEvent event, size_t idx);

signals:
    void storeCurrentPointToPointPool(const TargetPointInfo& point);

private:
    Ui::ControlPad *ui_;
    QTimer* timer_;
    JointsPosition current_position_;
    SettingPanel* setting_panel_;
};

#endif // CONTROL_PAD_H
