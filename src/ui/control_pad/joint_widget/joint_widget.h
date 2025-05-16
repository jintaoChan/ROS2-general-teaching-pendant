#ifndef JOINT_WIDGET_H
#define JOINT_WIDGET_H

#include <QWidget>
#include "ui_joint_widget.h"

namespace Ui {
class JointWidget;
}

class JointWidget : public QWidget
{
    Q_OBJECT

public:
    explicit JointWidget(const std::string& jointName, QWidget *parent = nullptr);
    ~JointWidget();

public:
    void setJointPosition(double p);
    auto getJointPosition() const -> double;
    auto getJointName() -> std::string;

signals:
    void MoveButtonClicked(MoveButtonType type, MoveButtonEvent event);

private:
    Ui::JointWidget *ui;
};

#endif // JOINT_WIDGET_H
