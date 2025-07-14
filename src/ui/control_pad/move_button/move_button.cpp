#include "move_button.h"

MoveButton::MoveButton(QWidget *parent) :
    QPushButton(parent)
{

}

MoveButton::MoveButton(MoveButtonType type, QWidget *parent)
    : MoveButton(parent)
{
    setButtonType(type);
}


void MoveButton::setButtonType(MoveButtonType type)
{
    m_ButtonType = type;
    switch(type) {
    case MoveButtonType::BACKWARD_VELOCITY: setText("<<"); break;
    case MoveButtonType::BACKWARD_STEP: setText("<"); break;
    case MoveButtonType::FORWARD_STEP: setText(">"); break;
    case MoveButtonType::FORWARD_VELOCITY: setText(">>"); break;
    }
}

void MoveButton::setJointName(const std::string &jointName)
{
    m_JointName = jointName;
}

MoveButtonType MoveButton::getButtonType()
{
    return m_ButtonType;
}

