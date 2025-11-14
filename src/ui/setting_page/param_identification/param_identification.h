#ifndef PARAM_IDENTIFICATION_H
#define PARAM_IDENTIFICATION_H

#include <QWidget>
#include <optional>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "simulate_external_force_bar.h"

class ParamIdentification : public QWidget
{
    Q_OBJECT

public:
    explicit ParamIdentification(QWidget *parent = nullptr);
    ~ParamIdentification();

private:
    std::optional<trajectory_msgs::msg::JointTrajectory> loadTrajFile(const QString& path);

private slots:
    void copyInfoClicked();
    void identifyClicked();

private:
    QPushButton* copy_info_button_;
    QPushButton* identify_button_;
    QLabel* base_param_size_title_;
    QLabel* base_param_size_;
    QHBoxLayout* base_param_size_layout_;
    QVBoxLayout* layout_;
    QList<SimulateExternalForceBar*> sefb_list_;
};

#endif // PARAM_IDENTIFICATION_H
