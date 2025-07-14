#include "cartesian_pad.h"

CartesianPad::CartesianPad(QWidget *parent)
    : QWidget{parent}
{
    m_Layout = new QVBoxLayout(this);
    for(size_t i = 0 ; i < m_CartesianPad.size(); ++i) {
        auto& row = m_CartesianPad[i];
        row = new CartesianDOFRow(i, this);
        connect(row->BackwardVelocityButton,&MoveButton::pressed,[&]() { emit MoveButtonClicked(row->BackwardVelocityButton->getButtonType(), MoveButtonEvent::PRESSED, row->DOFIndex);});
        connect(row->BackwardVelocityButton,&MoveButton::released,[&]() { emit MoveButtonClicked(row->BackwardVelocityButton->getButtonType(), MoveButtonEvent::RELEASED, row->DOFIndex);});
        connect(row->ForwardVelocityButton,&MoveButton::pressed,[&]() { emit MoveButtonClicked(row->ForwardVelocityButton->getButtonType(), MoveButtonEvent::PRESSED, row->DOFIndex);});
        connect(row->ForwardVelocityButton,&MoveButton::released,[&]() { emit MoveButtonClicked(row->ForwardVelocityButton->getButtonType(), MoveButtonEvent::RELEASED, row->DOFIndex);});

        // connect(row->BackwardPositionButton,&MoveButton::clicked,[&]() { emit MoveButtonClicked(row->BackwardPositionButton->getButtonType(), MoveButtonEvent::CLICKED, row->DOFIndex);});
        // connect(row->ForwardPositionButton,&MoveButton::clicked,[&]() { emit MoveButtonClicked(row->ForwardPositionButton->getButtonType(), MoveButtonEvent::CLICKED, row->DOFIndex);});
        m_Layout->addWidget(row);
    }
    setLayout(m_Layout);
}
