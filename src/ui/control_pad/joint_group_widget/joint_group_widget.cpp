#include "joint_group_widget.h"
#include "ui_joint_group_widget.h"

JointGroupWidget::JointGroupWidget(const std::string& jointGroupName, QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::JointGroupWidget)
{
    ui->setupUi(this);
    ui->joint_group_name->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    ui->joint_group_name->setText(QString::fromStdString(jointGroupName));
}

JointGroupWidget::~JointGroupWidget()
{
    delete ui;
}

void JointGroupWidget::addJoint(const std::string &jointName)
{
    JointWidget* joint = new JointWidget(jointName, this);
    ui->verticalLayout->addWidget(joint);
    connect(joint, &JointWidget::MoveButtonClicked,[this, jointName](MoveButtonType type, MoveButtonEvent event){emit MoveButtonClicked(type, event, jointName);});
}

std::string JointGroupWidget::getJointGroupName() const
{
    return ui->joint_group_name->text().toStdString();
}
