#include "cartesian_pad.h"

CartesianPad::CartesianPad(QWidget *parent)
    : QWidget{parent}
{
    layout_ = new QVBoxLayout(this);
    for(size_t i = 0 ; i < m_CartesianPad.size(); ++i) {
        auto& row = m_CartesianPad[i];
        row = new CartesianDOFRow(i, this);
        connect(row->backward_velocity_button_,&MoveButton::pressed,[&]() { emit MoveButtonClicked(row->backward_velocity_button_->getButtonType(), MoveButtonEvent::PRESSED, row->dof_index_);});
        connect(row->backward_velocity_button_,&MoveButton::released,[&]() { emit MoveButtonClicked(row->backward_velocity_button_->getButtonType(), MoveButtonEvent::RELEASED, row->dof_index_);});
        connect(row->forward_velocity_button_,&MoveButton::pressed,[&]() { emit MoveButtonClicked(row->forward_velocity_button_->getButtonType(), MoveButtonEvent::PRESSED, row->dof_index_);});
        connect(row->forward_velocity_button_,&MoveButton::released,[&]() { emit MoveButtonClicked(row->forward_velocity_button_->getButtonType(), MoveButtonEvent::RELEASED, row->dof_index_);});

        layout_->addWidget(row);
    }
    setLayout(layout_);
}
