#include <magic_enum/magic_enum.hpp>
#include <QMessageBox>
#include "setting_panel.h"
#include "ui_setting_panel.h"
#include "kinematics_plugin.h"

SettingPanel::SettingPanel(QWidget *parent)
    : QWidget(parent)
    , ui_(new Ui::SettingPanel)
{
    ui_->setupUi(this);
    EmptyOkDoubleValidator * validator = new EmptyOkDoubleValidator(this);
    ui_->lineEdit_step->setValidator(validator);
    ui_->lineEdit_step->setFixedWidth(50);
    QObject::connect(ui_->lineEdit_step, &QLineEdit::editingFinished, [&](){
        if(ui_->lineEdit_step->text().isEmpty()) {
            ui_->lineEdit_step->setText(QString::number(0));
        }
    });
    constexpr auto coordEntries = magic_enum::enum_entries<ControlCoordinateSystemType>();
    for(const auto& coord : coordEntries) {
        ui_->coordinate_selection_combobox->addItem(QString::fromStdString(std::string(coord.second)));
    }
    QObject::connect(ui_->coordinate_selection_combobox, &QComboBox::currentTextChanged, [this](const auto& text) {
        auto coord_type = magic_enum::enum_cast<ControlCoordinateSystemType>(text.toStdString()).value();
        if (coord_type == ControlCoordinateSystemType::Tool && !RobotHandle::instance().isToolFrameSet()) {
            QMessageBox::warning(this, "Warning", "Tool is not set yet!");
            QSignalBlocker blocker(ui_->coordinate_selection_combobox);
            ui_->coordinate_selection_combobox->setCurrentIndex(previous_coordinate_index_);
        } else {
            previous_coordinate_index_ = ui_->coordinate_selection_combobox->currentIndex();
            KinematicsPlugin::instance().selectCoordinateSystem(coord_type);
        }
    });
    ui_->coordinate_selection_combobox->setCurrentText(QString::fromStdString(std::string(magic_enum::enum_name(ControlCoordinateSystemType::Base))));
    KinematicsPlugin::instance().selectCoordinateSystem(ControlCoordinateSystemType::Base);
    previous_coordinate_index_ = ui_->coordinate_selection_combobox->currentIndex();
    ui_->horizontalSlider_velocity->setMaximum(100);
    ui_->horizontalSlider_velocity->setMinimum(0);
    QObject::connect(ui_->horizontalSlider_velocity, &QSlider::valueChanged, [&](int value){ ui_->label_horizontalSlider_velocity->setText(QString::number(value) + QString(" %")); });
    ui_->horizontalSlider_velocity->setValue(1);
    ui_->horizontalSlider_velocity->setValue(0);
    ui_->drag_button->setEnabled(false);

    RobotHandle::instance().registerMotorStatusCallback([this](JointsStatus state) {
        QMetaObject::invokeMethod(this, [this, state]() {
            bool is_enabled = true;
            bool is_in_error = false;
            for(const auto& s: state) {
                if(s.second == DriverState::STATE_FAULT) {
                    is_in_error = true;
                }
                if(s.second != DriverState::STATE_OPERATION_ENABLED) {
                    is_enabled = false;
                }
            }
            is_enabled_ = is_enabled;
            is_in_error_ = is_in_error;
            ;
        }, Qt::QueuedConnection);
    });

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &SettingPanel::regularUpdate);
    timer_->start(20);
}

SettingPanel::~SettingPanel()
{
    delete ui_;
}

double SettingPanel::getStep() const
{
    return ui_->lineEdit_step->text().toDouble();
}

double SettingPanel::getVelocity() const
{
    return (double)ui_->horizontalSlider_velocity->value() / 100;
}

ControlCoordinateSystemType SettingPanel::getCoordinateSystem()
{
    return control_coordinate_system_type_;
}

void SettingPanel::regularUpdate()
{
    const auto& is_running = RobotHandle::instance().isRunning();
    QString styleSheet;
    if(is_running) {
        styleSheet = R"(
            QLabel#running_state_label {
                border-radius: 10px;
                border: 1px solid black;
                background-color: yellow;
            }
        )";
    }
    else{
        styleSheet = R"(
            QLabel#running_state_label {
                border-radius: 10px;
                border: 1px solid black;
                background-color: green;
            }
        )";
    }
    ui_->running_state_label->setStyleSheet(styleSheet);

    if(is_enabled_) {
        ui_->motor_driver_enable_button->setText("Disable");
    }
    else{
        ui_->motor_driver_enable_button->setText("Enable");
    }
    if(is_in_error_) {
        ui_->clear_fault_button->setEnabled(true);
    }
    else{
        ui_->clear_fault_button->setEnabled(false);
    }
    const auto& is_dragging = KinematicsPlugin::instance().isDragging();
    if(is_dragging) {
        ui_->drag_button->setChecked(true);
    }
    else{
        ui_->drag_button->setChecked(false);
    }
}

void SettingPanel::activateDrag()
{
    ui_->drag_button->setEnabled(true);
    ui_->drag_button->setCheckable(true);
}


void SettingPanel::on_coordinate_selection_combobox_currentIndexChanged(int)
{
    auto text = ui_->coordinate_selection_combobox->currentText();
    control_coordinate_system_type_ = magic_enum::enum_cast<ControlCoordinateSystemType>(text.toStdString()).value();
}

void SettingPanel::on_drag_button_toggled(bool checked)
{
    if(checked) {
        KinematicsPlugin::instance().startDragging();
    }
    else{
        KinematicsPlugin::instance().stopDragging();
    }
}

void SettingPanel::on_clear_fault_button_clicked()
{
    RobotHandle::instance().clearFault();
}

void SettingPanel::on_motor_driver_enable_button_clicked()
{
    if(is_enabled_) {
        RobotHandle::instance().disableMotorDrive();
    }
    else{
        RobotHandle::instance().enableMotorDrive();
    }
}

