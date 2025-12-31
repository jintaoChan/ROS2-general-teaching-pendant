#ifndef SIMULATE_EXTERNAL_FORCE_BAR_H
#define SIMULATE_EXTERNAL_FORCE_BAR_H

#include <QWidget>
#include <QHBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QLineEdit>

class SimulateExternalForceBar : public QWidget
{
    Q_OBJECT
public:
    explicit SimulateExternalForceBar(const QString& joint_name, QWidget *parent = nullptr);

signals:

private slots:
    void whenSliderMoved(int position);

private:
    QHBoxLayout* layout_;
    QString joint_name_;
    QLabel* joint_name_label_;
    QSlider* slider_;
    QLineEdit* torque_editor_;

};

#endif // SIMULATE_EXTERNAL_FORCE_BAR_H
