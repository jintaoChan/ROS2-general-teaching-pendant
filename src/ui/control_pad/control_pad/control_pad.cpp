#include <QVBoxLayout>
#include <QHBoxLayout>
#include "control_pad.h"
#include "ui_control_pad.h"
#include "joint_group_widget.h"
#include "robot_description.h"
#include "controller_switcher.h"
#include "mode_changer.h"
#include "cartesian_controller.h"
#include "move_executor.h"

ControlPad::ControlPad(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ControlPad)
    , m_SettingPanel(this)
{
    ui->setupUi(this);
    m_Node = rclcpp::Node::make_shared("control_pad");
    m_JointStateSubscription = m_Node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&ControlPad::recvCallback, this, std::placeholders::_1));
    auto& robotDes = RobotDescription::instance();
    auto jointGroupsNames = robotDes.getJointGroupsNames();
    QHBoxLayout* hlayout = new QHBoxLayout(this);
    auto cartesianPad = new CartesianPad(this);
    connect(cartesianPad, &CartesianPad::MoveButtonClicked, this, [this](MoveButtonType type, MoveButtonEvent event, size_t idx){emit MoveCommander(type, event, idx);});
    hlayout->addWidget(cartesianPad);
    QVBoxLayout* vlayout = new QVBoxLayout(this);
    for(const auto& jgName: jointGroupsNames) {
        JointGroupWidget* jg = new JointGroupWidget(jgName, this);
        auto joint = robotDes.getJointGroupJointNames(jgName);
        for(const auto& jt : joint) {
            jg->addJoint(jt);
            connect(jg, &JointGroupWidget::MoveButtonClicked, [this](MoveButtonType type, MoveButtonEvent event, const std::string& jointName){emit MoveCommander(type, event, jointName);});
        }
        vlayout->addWidget(jg);
    }
    hlayout->addLayout(vlayout);

    vlayout->addWidget(&m_SettingPanel);
    setLayout(hlayout);

    connect(&m_SettingPanel, &SettingPanel::on_add_to_point_pool_button_clicked, this, [this]() {
        MovePointInfo p;
        for(const auto& jointGroup: this->findChildren<JointGroupWidget*>()) {
            p[jointGroup->getJointGroupName()].MoveType = MoveTypeEnum::JOINT;
            for(const auto& joint: jointGroup->findChildren<JointWidget*>()) {
                p[jointGroup->getJointGroupName()].JointNames.push_back(joint->getJointName());
                p[jointGroup->getJointGroupName()].Values.push_back(joint->getJointPosition());
            }
        }
        emit storeCurrentPointToPointPool(p);
    });

}

void ControlPad::run(){
    rclcpp::WallRate loop_rate(100);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(m_Node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
}

void ControlPad::MoveCommander(MoveButtonType type, MoveButtonEvent event, const std::string &jointName)
{
    ControllerSwitcher::instance().switchToControlPad();
    auto& modeChanger = ModeChanger::instance();
    auto& moveExecutor = MoveExecutor::instance();
    CartesianController::instance().dropControl();

    sensor_msgs::msg::JointState jointMsg;
    jointMsg.name.push_back(jointName);
    switch(type){
    case MoveButtonType::BACKWARD_VELOCITY: {
        modeChanger.changeToVelocityMode();
        if(event == MoveButtonEvent::PRESSED){
            jointMsg.velocity.push_back(-m_SettingPanel.getVelocity());
        }
        else if(event == MoveButtonEvent::RELEASED){
            jointMsg.velocity.push_back(0);
        }
        break;
    }
    case MoveButtonType::BACKWARD_STEP: {
        modeChanger.changeToPositionMode();
        if(event == MoveButtonEvent::CLICKED)
            jointMsg.position.push_back(m_CurrentPosition[jointName] - m_SettingPanel.getStep());
        break;
    }
    case MoveButtonType::FORWARD_STEP: {
        modeChanger.changeToPositionMode();
        if(event == MoveButtonEvent::CLICKED)
            jointMsg.position.push_back(m_CurrentPosition[jointName] + m_SettingPanel.getStep());
        break;
    }
    case MoveButtonType::FORWARD_VELOCITY: {
        modeChanger.changeToVelocityMode();
        if(event == MoveButtonEvent::PRESSED){
            jointMsg.velocity.push_back(m_SettingPanel.getVelocity());
        }
        else if(event == MoveButtonEvent::RELEASED){
            jointMsg.velocity.push_back(0);
        }
        break;
    }
    }
    moveExecutor.pubMoveCommand(jointMsg);
}

void ControlPad::MoveCommander(MoveButtonType type, MoveButtonEvent event, size_t idx) {
    ControllerSwitcher::instance().switchToControlPad();
    ModeChanger::instance().changeToVelocityMode();
    CartesianController::instance().takeControl();
    std::array<double, 6> arr{0,0,0,0,0,0};
    switch(type){
    case MoveButtonType::BACKWARD_VELOCITY: {
        if(event == MoveButtonEvent::PRESSED){
            arr[idx] = -m_SettingPanel.getVelocity();
        }
        else if(event == MoveButtonEvent::RELEASED){
        }
        CartesianController::instance().setVelocity(arr);
        break;
    }
    case MoveButtonType::FORWARD_VELOCITY: {
        if(event == MoveButtonEvent::PRESSED){
            arr[idx] = m_SettingPanel.getVelocity();
        }
        else if(event == MoveButtonEvent::RELEASED){
        }
        CartesianController::instance().setVelocity(arr);
        break;
    }
    }

}

ControlPad::~ControlPad()
{
    delete ui;
}

void ControlPad::recvCallback(const sensor_msgs::msg::JointState &msg)
{
    for(const auto& jointGroup: this->findChildren<JointGroupWidget*>()) {
        for(const auto& joint: jointGroup->findChildren<JointWidget*>()) {
            auto index = std::find(msg.name.begin(), msg.name.end(), joint->getJointName()) - msg.name.begin();
            joint->setJointPosition(msg.position.at(index));
            m_CurrentPosition[joint->getJointName()] = msg.position.at(index);
        }
    }

    auto position = CartesianController::instance().getPosition();
    auto rotation = CartesianController::instance().getRotation();
    auto dofRows = this->findChildren<CartesianDOFRow*>();
    for(size_t i = 0;i < position.size(); ++i) {
        dofRows[i]->Position->setText(QString::number(position[i]));
    }
    for(size_t i = 0;i < rotation.size(); ++i) {
        dofRows[i + 3]->Position->setText(QString::number(rotation[i]));
    }

}
