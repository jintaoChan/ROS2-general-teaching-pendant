#include "cartesian_dof_row.h"

CartesianDOFRow::CartesianDOFRow(size_t idx, QWidget *parent)
    : QWidget{parent}, dof_index_(idx)
{
    layout_ = new QHBoxLayout(this);
    backward_velocity_button_ = new MoveButton(MoveButtonType::BACKWARD_VELOCITY, this);
    // BackwardPositionButton = new MoveButton(MoveButtonType::BACKWARD_STEP, this);
    // ForwardPositionButton = new MoveButton(MoveButtonType::FORWARD_STEP, this);
    forward_velocity_button_ = new MoveButton(MoveButtonType::FORWARD_VELOCITY, this);
    position_label_ = new QLabel("0");
    position_label_->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    layout_->addWidget(backward_velocity_button_);
    // layout_->addWidget(BackwardPositionButton);
    layout_->addWidget(position_label_);
    // layout_->addWidget(ForwardPositionButton);
    layout_->addWidget(forward_velocity_button_);
}
