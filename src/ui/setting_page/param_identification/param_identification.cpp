#include <QInputDialog>
#include <QMessageBox>
#include <fstream>
#include <sstream>
#include "robot_handle.h"
#include "dynamic_plugin.h"
#include "param_identification.h"
#include "ui_param_identification.h"

ParamIdentification::ParamIdentification(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ParamIdentification)
{
    ui->setupUi(this);
}

ParamIdentification::~ParamIdentification()
{
    delete ui;
}

std::optional<trajectory_msgs::msg::JointTrajectory> ParamIdentification::loadTrajFile(const QString &path)
{
    trajectory_msgs::msg::JointTrajectory traj;
    std::ifstream file(path.toStdString(), std::ios::in | std::ios::binary);
    if (!file) {
        QMessageBox::warning(this, "Warning", "Cannot open file: " + path);
        return std::nullopt;
    }
    std::string line;
    auto period = RobotHandle::instance().getControllerUpdatePeriod();
    size_t index{1};
    traj.joint_names = RobotHandle::instance().getJointsName();
    trajectory_msgs::msg::JointTrajectoryPoint last_p;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        trajectory_msgs::msg::JointTrajectoryPoint p;
        double val;
        while (iss >> val) {
            p.positions.push_back(val);
        }
        if(p.positions.size() != traj.joint_names.size()) {
            QMessageBox::warning(this, "Warning", "The number of joint does not match!");
            return std::nullopt;
        }
        if(index > 1) {
            for(size_t i = 0;i < p.positions.size(); ++i) {
                p.velocities.push_back((p.positions[i] - last_p.positions[i]) / (period / 1e3));
            }
        }
        else{
            for(size_t i = 0;i < p.positions.size(); ++i) {
                p.velocities.push_back(0);
            }
        }
        p.time_from_start.sec = period * index / 1e3;
        p.time_from_start.nanosec = (uint32_t(period * index) % 1000) * 1e6;
        ++index;

        traj.points.push_back(p);
        last_p = p;
    }
    if(traj.points.empty()) {
        QMessageBox::warning(this, "Warning", "Emtpy file!");
        return std::nullopt;
    }
    return traj;
}

void ParamIdentification::on_copy_all_info_button_2_clicked()
{
    bool ok;
    auto path = QInputDialog::getText(this, tr("Excitation trajectory file"),
                                      tr("Please input the path"), QLineEdit::Normal,
                                      "", &ok);
    if(ok){
        auto traj_opt = loadTrajFile(path);
        if(traj_opt.has_value()) {
            RobotHandle::instance().moveJointByAbsPosition(traj_opt.value());
        }
        ui->base_param_size_label->setText(QString::number(DynamicPlugin::instance().getBaseParams().size()));
    }
}

