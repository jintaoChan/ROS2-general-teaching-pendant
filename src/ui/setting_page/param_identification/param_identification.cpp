#include "param_identification.h"
#include <QApplication>
#include <QInputDialog>
#include <QMessageBox>
#include <fstream>
#include <sstream>
#include <QClipboard>
#include <QProgressDialog>
#include <QPointer>
#include <thread>
#include <chrono>
#include <cereal/archives/json.hpp>
#include <stdexcept>

#include "database.h"
#include "motion_plugin.h"

ParamIdentification::ParamIdentification(const AppPorts& ports, QWidget *parent)
    :
    QWidget(parent),
    ports_(ports),
    copy_info_button_(new QPushButton(this)),
    load_param_button_(new QPushButton(this)),
    identify_button_(new QPushButton(this)),
    base_param_size_title_(new QLabel(this)),
    base_param_size_(new QLabel(this)),
    base_param_size_layout_(new QHBoxLayout()),
    layout_(new QVBoxLayout())
{
    if (ports_.state == nullptr || ports_.command == nullptr) {
        throw std::invalid_argument("ParamIdentification requires non-null state and command ports");
    }

    base_param_size_title_->setText("Base param size: ");
    base_param_size_layout_->addWidget(base_param_size_title_);
    base_param_size_layout_->addWidget(base_param_size_);
    copy_info_button_->setText("Save param to file");
    load_param_button_->setText("Load param from file");
    identify_button_->setText("Start identify by a trajectory");
    layout_->addLayout(base_param_size_layout_);
    layout_->addWidget(copy_info_button_);
    layout_->addWidget(load_param_button_);
    layout_->addWidget(identify_button_);
    setLayout(layout_);
    connect(copy_info_button_, &QPushButton::pressed, this, &ParamIdentification::saveInfoClicked);
    connect(load_param_button_, &QPushButton::pressed, this, &ParamIdentification::loadParamClicked);
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
    auto period = ports_.state->getControllerUpdatePeriod();
    size_t index{1};
    traj.joint_names = ports_.state->getJointsName();
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
                p.velocities.push_back((p.positions[i] - last_p.positions[i]) / (period / 1e9));
            }
        }
        else{
            for(size_t i = 0;i < p.positions.size(); ++i) {
                p.velocities.push_back(0);
            }
        }
        p.time_from_start.sec = period * index / 1e9;
        p.time_from_start.nanosec = (period * index) % 1000000000;
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

void ParamIdentification::saveInfoClicked()
{
    bool ok;
    auto path = QInputDialog::getText(this, tr("Save base param file"),
                                      tr("Please input the path"), QLineEdit::Normal,
                                      "", &ok);
    if(ok){
        std::ofstream file(path.toStdString(), std::ios::out | std::ios::trunc);
        std::vector<double> param_vec;
        if (!file) {
            QMessageBox::warning(this, "Warning", "Cannot open file: " + path);
            return;
        }
        cereal::JSONOutputArchive ar_out(file);
        const auto& base = MotionPlugin::instance().getDynamicsBaseParams();
        const auto& friction = MotionPlugin::instance().getDynamicsFrictionParams();
        const auto& Pb = MotionPlugin::instance().getDynamicsDepPb();
        const auto& Pd = MotionPlugin::instance().getDynamicsDepPd();
        const auto& Kd = MotionPlugin::instance().getDynamicsDepKd();
        ar_out(CEREAL_NVP(base), CEREAL_NVP(friction), CEREAL_NVP(Pb), CEREAL_NVP(Pd), CEREAL_NVP(Kd));
    }
}

void ParamIdentification::loadParamClicked()
{
    bool ok;
    auto path = QInputDialog::getText(this, tr("Base param file"),
                                      tr("Please input the path"), QLineEdit::Normal,
                                      "", &ok);
    if(ok){
        std::ifstream file(path.toStdString(), std::ios::in);
        std::string line;
        if (!file) {
            QMessageBox::warning(this, "Warning", "Cannot open file: " + path);
            return;
        }
        cereal::JSONInputArchive ar_in(file);
        Eigen::MatrixXd base, friction, Pb, Pd, Kd;
        ar_in(CEREAL_NVP(base), CEREAL_NVP(friction), CEREAL_NVP(Pb), CEREAL_NVP(Pd), CEREAL_NVP(Kd));
        MotionPlugin::instance().setDynamicsParams(base, friction, Pb, Pd, Kd);
        if(MotionPlugin::instance().isDynamicsReady()) {
            emit(identifyFinished());
            std::cout << "Ready to drag!" << std::endl;
        }
        else {
            QMessageBox::warning(this, "Warning", "Invalid param file!");
        }
    }
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
            // run the heavy identification in a worker thread while showing a
            // modal progress dialog that prevents interaction with the main window
            QPointer<QProgressDialog> progress = new QProgressDialog("Identification in progress...", QString(), 0, 0, this);
            progress->setWindowModality(Qt::ApplicationModal);
            progress->setCancelButton(nullptr);
            progress->setWindowTitle(tr("Please wait"));
            progress->setMinimumDuration(0);
            progress->show();

            // copy trajectory to worker-safe object
            auto traj = traj_opt.value();

            std::thread worker([this, traj, progress]() mutable {
                // sample indices
                auto sample_start_index = DataBase::instance().getCurrentIndex();
                decltype(sample_start_index) sample_end_index;

                // publish trajectory
                ports_.command->executeTrajectory(traj);

                // wait until it reports finished; poll with sleep
                while (!ports_.state->isTrajectoryComplete()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }

                sample_end_index = DataBase::instance().getCurrentIndex();
                MotionPlugin::instance().identify(sample_start_index, sample_end_index);
                std::cout << "Using points from " << sample_start_index << " to " << sample_end_index << std::endl;
                // update UI on the GUI thread
                if (progress) {
                    QMetaObject::invokeMethod(progress, [progress]() {
                        if (progress) progress->close();
                    }, Qt::QueuedConnection);
                }
                QMetaObject::invokeMethod(qApp, [this]() {
                    auto size = MotionPlugin::instance().getDynamicsBaseParams().size() + MotionPlugin::instance().getDynamicsFrictionParams().size();
                    base_param_size_->setText(QString::number(size));
                    emit(identifyFinished());
                }, Qt::QueuedConnection);
            });
            worker.detach();
        }
    }
}

