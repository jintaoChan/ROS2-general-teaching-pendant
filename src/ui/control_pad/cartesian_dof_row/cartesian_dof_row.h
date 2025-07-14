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
    MoveButton* BackwardVelocityButton;
    // MoveButton* BackwardPositionButton;
    // MoveButton* ForwardPositionButton;
    MoveButton* ForwardVelocityButton;
    QLabel* Position;
    size_t DOFIndex;

private:
    QHBoxLayout* m_Layout;
};

#endif // CARTESIAN_DOF_ROW_H
