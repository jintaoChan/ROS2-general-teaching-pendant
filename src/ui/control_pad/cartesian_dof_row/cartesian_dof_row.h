#ifndef CARTESIAN_DOF_ROW_H
#define CARTESIAN_DOF_ROW_H

#include <QWidget>
#include "move_button.h"
#include <QHBoxLayout>
#include <QLabel>

class CartesianDOFRow : public QWidget
{
    Q_OBJECT
public:
    explicit CartesianDOFRow(size_t dofIdx, QWidget *parent = nullptr);

signals:

public:
    MoveButton* backward_velocity_button_;
    // MoveButton* BackwardPositionButton;
    // MoveButton* ForwardPositionButton;
    MoveButton* forward_velocity_button_;
    QLabel* position_label_;
    size_t dof_index_;

private:
    QHBoxLayout* layout_;
};

#endif // CARTESIAN_DOF_ROW_H
