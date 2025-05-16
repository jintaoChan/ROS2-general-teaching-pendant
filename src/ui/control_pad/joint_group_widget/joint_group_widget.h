#ifndef JOINT_GROUP_WIDGET_H
#define JOINT_GROUP_WIDGET_H

#include <QWidget>
#include "joint_widget.h"

namespace Ui {
class JointGroupWidget;
}

class JointGroupWidget : public QWidget
{
    Q_OBJECT

public:
    explicit JointGroupWidget(const std::string& jointGroupName, QWidget *parent = nullptr);
    ~JointGroupWidget();

public:
    void addJoint(const std::string& jointName);
    auto getJointGroupName() const -> std::string;

signals:
    void MoveButtonClicked(MoveButtonType type, MoveButtonEvent event, const std::string& jointName);

private:
    Ui::JointGroupWidget *ui;
};

#endif // JOINT_GROUP_WIDGET_H
