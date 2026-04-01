#include <QtConcurrent/QtConcurrent>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <stdexcept>

#include "control_pad.h"
#include "ui_control_pad.h"
#include "joint_group_widget.h"
#include "setting_panel.h"
#include "motion_plugin.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

ControlPad::ControlPad(SettingPanel* setting_panel, IRobotStateProvider* state_port, QWidget *parent)
    : QWidget(parent)
    , ui_(new Ui::ControlPad)
    , setting_panel_(setting_panel)
    , state_port_(state_port)
{
    if (state_port_ == nullptr) {
        throw std::invalid_argument("ControlPad requires non-null state port");
    }

    ui_->setupUi(this);
    QHBoxLayout* hlayout = new QHBoxLayout(this);
    auto cartesianPad = new CartesianPad(this);
    connect(cartesianPad, &CartesianPad::MoveButtonClicked, this, [this](MoveButtonType type, MoveButtonEvent event, size_t idx){emit MoveCommander(type, event, idx);});
    hlayout->addWidget(cartesianPad);
    QVBoxLayout* vlayout = new QVBoxLayout();

    JointGroupWidget* jg = new JointGroupWidget("tmp group", this);
    auto joints = state_port_->getJointsName();
    for(const auto& jt : joints) {
        jg->addJoint(jt);
        connect(jg, &JointGroupWidget::MoveButtonClicked, [this](MoveButtonType type, MoveButtonEvent event, const std::string& joint_name){emit MoveCommander(type, event, joint_name);});
    }
    vlayout->addWidget(jg);
    hlayout->addLayout(vlayout);

    connect(setting_panel_, &SettingPanel::on_record_point_button_clicked, this, [this]() {
        TargetPointInfo p;
        for(const auto& jointGroup: this->findChildren<JointGroupWidget*>()) {
            p.MoveType = MoveTypeEnum::JOINT;
            for(const auto& joint: jointGroup->findChildren<JointWidget*>()) {
                p.JointValues[joint->getJointName()].joint_value = joint->getJointPosition();
            }
        }
        emit storeCurrentPointToPointPool(p);
    });


    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &ControlPad::regularUpdate);
    // Cap UI refresh at ~125 Hz (8ms) to avoid wasting CPU on invisible redraws
    int period_ms = std::max(8, static_cast<int>(state_port_->getControllerUpdatePeriod() / 1e6));
    timer_->start(period_ms);
}

void ControlPad::MoveCommander(MoveButtonType type, MoveButtonEvent event, const std::string &joint_name) const {
    const auto& velo_ratio = setting_panel_->getVelocity();
    switch(type){
    case MoveButtonType::BACKWARD_VELOCITY: {
        if(event == MoveButtonEvent::PRESSED){
            MotionPlugin::instance().jogJoint(joint_name, -1.0, velo_ratio);
        }
        else if(event == MoveButtonEvent::RELEASED){
            MotionPlugin::instance().stopStreaming();
        }
        break;
    }
    case MoveButtonType::BACKWARD_STEP: {
        if(event == MoveButtonEvent::CLICKED) {
            JointsPosition joint_position;
            joint_position[joint_name].joint_value = current_position_.at(joint_name).joint_value - setting_panel_->getStep();
            MotionPlugin::instance().moveToJointTarget(joint_position, velo_ratio);
        }
        break;
    }
    case MoveButtonType::FORWARD_STEP: {
        if(event == MoveButtonEvent::CLICKED) {
            JointsPosition joint_position;
            joint_position[joint_name].joint_value = current_position_.at(joint_name).joint_value + setting_panel_->getStep();
            MotionPlugin::instance().moveToJointTarget(joint_position, velo_ratio);
        }
        break;
    }
    case MoveButtonType::FORWARD_VELOCITY: {
        if(event == MoveButtonEvent::PRESSED){
            MotionPlugin::instance().jogJoint(joint_name, 1.0, velo_ratio);
        }
        else if(event == MoveButtonEvent::RELEASED){
            MotionPlugin::instance().stopStreaming();
        }
        break;
    }
    }
}

void ControlPad::MoveCommander(MoveButtonType type, MoveButtonEvent event, size_t idx) {
    std::array<double, 6> arr{0,0,0,0,0,0};
    switch(type){
    case MoveButtonType::BACKWARD_VELOCITY: {
        if(event == MoveButtonEvent::PRESSED){
            arr[idx] = -setting_panel_->getVelocity() * state_port_->getCartesianLimitsMaxTransVel();
        }
        else if(event == MoveButtonEvent::RELEASED){
        }
        break;
    }
    case MoveButtonType::FORWARD_VELOCITY: {
        if(event == MoveButtonEvent::PRESSED){
            arr[idx] = setting_panel_->getVelocity() * state_port_->getCartesianLimitsMaxTransVel();
        }
        else if(event == MoveButtonEvent::RELEASED){
        }
        break;
    default:
        break;
    }
    }
    MotionPlugin::instance().jogCartesian(arr);
}

ControlPad::~ControlPad()
{
    delete ui_;
}

void ControlPad::regularUpdate()
{
    auto current_joint_position = state_port_->getCurrentJointPosition();

    for(const auto& jointGroup: this->findChildren<JointGroupWidget*>()) {
        for(const auto& joint: jointGroup->findChildren<JointWidget*>()) {
            auto pos = current_joint_position[joint->getJointName()].joint_value;
            joint->setJointPosition(pos);
            current_position_[joint->getJointName()].joint_value = pos;
        }
    }

    auto pose = MotionPlugin::instance().getCurrentCartesianPose();
    auto dofRows = this->findChildren<CartesianDOFRow*>();
    dofRows[0]->position_label_->setText(QString::number(pose.pose.position.x, 'f', 3));
    dofRows[1]->position_label_->setText(QString::number(pose.pose.position.y, 'f', 3));
    dofRows[2]->position_label_->setText(QString::number(pose.pose.position.z, 'f', 3));

    tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    double rot_x, rot_y, rot_z;
    tf2::Matrix3x3(q).getRPY(rot_x, rot_y, rot_z);

    dofRows[3]->position_label_->setText(QString::number(rot_x, 'f', 3));
    dofRows[4]->position_label_->setText(QString::number(rot_y, 'f', 3));
    dofRows[5]->position_label_->setText(QString::number(rot_z, 'f', 3));
}
