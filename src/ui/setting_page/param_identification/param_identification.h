#ifndef PARAM_IDENTIFICATION_H
#define PARAM_IDENTIFICATION_H

#include <QWidget>
#include <optional>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace Ui {
class ParamIdentification;
}

class ParamIdentification : public QWidget
{
    Q_OBJECT

public:
    explicit ParamIdentification(QWidget *parent = nullptr);
    ~ParamIdentification();

private:
    std::optional<trajectory_msgs::msg::JointTrajectory> loadTrajFile(const QString& path);

private slots:
    void on_copy_all_info_button_2_clicked();

private:
    Ui::ParamIdentification *ui;
};

#endif // PARAM_IDENTIFICATION_H
