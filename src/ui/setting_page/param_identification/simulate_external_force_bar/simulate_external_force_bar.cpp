#include "simulate_external_force_bar.h"
#include "robot_handle.h"

SimulateExternalForceBar::SimulateExternalForceBar(const QString& joint_name, QWidget *parent)
    :
    QWidget{parent},
    layout_{new QHBoxLayout()},
    joint_name_{joint_name},
    joint_name_label_{new QLabel(this)},
    slider_{new QSlider(Qt::Horizontal, this)},
    torque_editor_{new QLineEdit(this)}
{
    joint_name_label_->setText(joint_name_);
    slider_->setRange(-20,20);
    torque_editor_->setText(QString::number(0));
    torque_editor_->setReadOnly(true);
    layout_->addWidget(joint_name_label_);
    layout_->addWidget(slider_);
    layout_->addWidget(torque_editor_);
    setLayout(layout_);
    connect(slider_, &QSlider::valueChanged, this, &SimulateExternalForceBar::whenSliderMoved);
}

void SimulateExternalForceBar::whenSliderMoved(int position)
{
    RobotHandle::instance().setJointTorqueOffset(joint_name_.toStdString(), position);
    torque_editor_->setText(QString::number(position));

}
