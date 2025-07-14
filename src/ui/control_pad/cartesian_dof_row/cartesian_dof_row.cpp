#include "cartesian_dof_row.h"

CartesianDOFRow::CartesianDOFRow(size_t idx, QWidget *parent)
    : QWidget{parent}, DOFIndex(idx)
{
    m_Layout = new QHBoxLayout(this);
    BackwardVelocityButton = new MoveButton(MoveButtonType::BACKWARD_VELOCITY, this);
    // BackwardPositionButton = new MoveButton(MoveButtonType::BACKWARD_STEP, this);
    // ForwardPositionButton = new MoveButton(MoveButtonType::FORWARD_STEP, this);
    ForwardVelocityButton = new MoveButton(MoveButtonType::FORWARD_VELOCITY, this);
    Position = new QLabel("0");
    m_Layout->addWidget(BackwardVelocityButton);
    // m_Layout->addWidget(BackwardPositionButton);
    m_Layout->addWidget(Position);
    // m_Layout->addWidget(ForwardPositionButton);
    m_Layout->addWidget(ForwardVelocityButton);
}
