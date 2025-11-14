#include <QApplication>
#include <QInputDialog>
#include <QMessageBox>
#include <fstream>
#include <sstream>
#include <QClipboard>
#include "robot_handle.h"
#include "database.h"
#include "dynamic_plugin.h"
#include "param_identification.h"
#include "robot_handle.h"

ParamIdentification::ParamIdentification(QWidget *parent)
    :
    QWidget(parent),
    copy_info_button_(new QPushButton(this)),
    identify_button_(new QPushButton(this)),
    base_param_size_title_(new QLabel(this)),
    base_param_size_(new QLabel(this)),
    base_param_size_layout_(new QHBoxLayout()),
    layout_(new QVBoxLayout())
{
    base_param_size_title_->setText("Base param size: ");
    base_param_size_layout_->addWidget(base_param_size_title_);
    base_param_size_layout_->addWidget(base_param_size_);
    copy_info_button_->setText("Copy base param to clip board");
    identify_button_->setText("Start identify by a trajectory");
    layout_->addLayout(base_param_size_layout_);
    for(const auto& n : RobotHandle::instance().getJointsName()) {
        auto sefb = new SimulateExternalForceBar(QString::fromStdString(n), this);
        sefb_list_.push_back(sefb);
        layout_->addWidget(sefb);
    }
    layout_->addWidget(copy_info_button_);
    layout_->addWidget(identify_button_);
    setLayout(layout_);
    connect(copy_info_button_, &QPushButton::pressed, this, &ParamIdentification::copyInfoClicked);
    connect(identify_button_, &QPushButton::pressed, this, &ParamIdentification::identifyClicked);

}

ParamIdentification::~ParamIdentification()
{
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

void ParamIdentification::copyInfoClicked()
{
    std::cout << DynamicPlugin::instance().getBaseParams() << std::endl;
}

void ParamIdentification::identifyClicked()
{
    bool ok;
    auto path = QInputDialog::getText(this, tr("Excitation trajectory file"),
                                      tr("Please input the path"), QLineEdit::Normal,
                                      "", &ok);
    if(ok){
        auto traj_opt = loadTrajFile(path);
        if(traj_opt.has_value()) {
            auto sample_start_index = DataBase::instance().getCurrentIndex();
            decltype(sample_start_index) sample_end_index;
            RobotHandle::instance().moveJointByAbsPosition(traj_opt.value());
            while(RobotHandle::instance().isRunning()){
            }
            sample_end_index = DataBase::instance().getCurrentIndex();
            DynamicPlugin::instance().identify(sample_start_index, sample_end_index);
        }
        base_param_size_->setText(QString::number(DynamicPlugin::instance().getBaseParams().size()));
    }
}

