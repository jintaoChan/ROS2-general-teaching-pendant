#include "joint_widget.h"

JointWidget::JointWidget(const std::string& jointName, QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::JointWidget)
{
    ui->setupUi(this);
    ui->joint_name->setText(QString::fromStdString(jointName));
    ui->joint_position->setMinimumWidth(100);
    ui->backward_velocity->setButtonType(MoveButtonType::BACKWARD_VELOCITY);
    ui->backward_step->setButtonType(MoveButtonType::BACKWARD_STEP);
    ui->forward_step->setButtonType(MoveButtonType::FORWARD_STEP);
    ui->forward_velocity->setButtonType(MoveButtonType::FORWARD_VELOCITY);

    connect(ui->backward_velocity,&MoveButton::pressed,[this]() { emit MoveButtonClicked(ui->backward_velocity->getButtonType(), MoveButtonEvent::PRESSED);});
    connect(ui->backward_velocity,&MoveButton::released,[this]() { emit MoveButtonClicked(ui->backward_velocity->getButtonType(), MoveButtonEvent::RELEASED);});
    connect(ui->backward_step,&MoveButton::clicked,[this]() { emit MoveButtonClicked(ui->backward_step->getButtonType(), MoveButtonEvent::CLICKED);});
    connect(ui->forward_step,&MoveButton::clicked,[this]() { emit MoveButtonClicked(ui->forward_step->getButtonType(), MoveButtonEvent::CLICKED);});
    connect(ui->forward_velocity,&MoveButton::pressed,[this]() { emit MoveButtonClicked(ui->forward_velocity->getButtonType(), MoveButtonEvent::PRESSED);});
    connect(ui->forward_velocity,&MoveButton::released,[this]() { emit MoveButtonClicked(ui->forward_velocity->getButtonType(), MoveButtonEvent::RELEASED);});
}

JointWidget::~JointWidget()
{
    delete ui;
}

void JointWidget::setJointPosition(double p)
{
    ui->joint_position->setText(QString::number(p, 'f', 3));
}

double JointWidget::getJointPosition() const
{
    return ui->joint_position->text().toDouble();
}

std::string JointWidget::getJointName()
{
    return ui->joint_name->text().toStdString();
}
