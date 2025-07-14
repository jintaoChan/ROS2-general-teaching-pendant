#ifndef MOVE_BUTTON_H
#define MOVE_BUTTON_H

#include <QPushButton>
#include <rclcpp/rclcpp.hpp>

namespace Ui {
class MoveButton;
}

enum class MoveButtonType : char {
    BACKWARD_VELOCITY = 0,
    BACKWARD_STEP,
    FORWARD_STEP,
    FORWARD_VELOCITY
};

enum class MoveButtonEvent : char {
    PRESSED = 0,
    RELEASED,
    CLICKED
};

class MoveButton : public QPushButton
{
    Q_OBJECT

public:
    explicit MoveButton(QWidget *parent = nullptr);
    explicit MoveButton(MoveButtonType type, QWidget *parent = nullptr);
    ~MoveButton(){};


public:
    void setButtonType(MoveButtonType type);
    void setJointName(const std::string& jointName);

    MoveButtonType getButtonType();


private:
    Ui::MoveButton *ui;
    MoveButtonType m_ButtonType;
    std::string m_JointName;
};

#endif // MOVE_BUTTON_H
