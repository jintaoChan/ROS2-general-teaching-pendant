#ifndef CARTESIAN_PAD_H
#define CARTESIAN_PAD_H

#include <QWidget>
#include <QVBoxLayout>
#include "cartesian_dof_row.h"

class CartesianPad : public QWidget
{
    Q_OBJECT
public:
    explicit CartesianPad(QWidget *parent = nullptr);

signals:
    void MoveButtonClicked(MoveButtonType type, MoveButtonEvent event, size_t idx);

private:
    QVBoxLayout* m_Layout;
    std::array<CartesianDOFRow*, 6> m_CartesianPad;
};

#endif // CARTESIAN_PAD_H
