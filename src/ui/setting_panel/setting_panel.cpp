#include <magic_enum/magic_enum.hpp>
#include "setting_panel.h"
#include "ui_setting_panel.h"
#include "kinematics_plugin.h"

SettingPanel::SettingPanel(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::SettingPanel)
{
    ui->setupUi(this);
    EmptyOkDoubleValidator * validator = new EmptyOkDoubleValidator(this);
    ui->lineEdit_step->setValidator(validator);
    ui->lineEdit_step->setFixedWidth(50);
    QObject::connect(ui->lineEdit_step, &QLineEdit::editingFinished, [&](){
        if(ui->lineEdit_step->text().isEmpty()) {
            ui->lineEdit_step->setText(QString::number(0));
        }
    });
    constexpr auto coordEntries = magic_enum::enum_entries<ControlCoordinateSystemType>();
    for(const auto& coord : coordEntries) {
        ui->coordinate_selection_combobox->addItem(QString::fromStdString(std::string(coord.second)));
    }
    QObject::connect(ui->coordinate_selection_combobox, &QComboBox::currentTextChanged, [](const auto& text) {
        KinematicsPlugin::instance().selectCoordinateSystem(magic_enum::enum_cast<ControlCoordinateSystemType>(text.toStdString()).value());
    });
    ui->coordinate_selection_combobox->setCurrentText(QString::fromStdString(std::string(magic_enum::enum_name(ControlCoordinateSystemType::Tool))));
    ui->horizontalSlider_velocity->setMaximum(100);
    ui->horizontalSlider_velocity->setMinimum(0);
    ui->label_horizontalSlider_velocity->setFixedWidth(50);
    QObject::connect(ui->horizontalSlider_velocity, &QSlider::valueChanged, [&](int value){ ui->label_horizontalSlider_velocity->setText(QString::number(value) + QString(" %")); });
    ui->horizontalSlider_velocity->setValue(1);
    ui->horizontalSlider_velocity->setValue(0);
}

SettingPanel::~SettingPanel()
{
    delete ui;
}

double SettingPanel::getStep() const
{
    return ui->lineEdit_step->text().toDouble();
}

double SettingPanel::getVelocity() const
{
    return (double)ui->horizontalSlider_velocity->value() / 100;
}

ControlCoordinateSystemType SettingPanel::getCoordinateSystem()
{
    return m_ControlCoordinateSystemType;
}


void SettingPanel::on_coordinate_selection_combobox_currentIndexChanged(int index)
{
    auto text = ui->coordinate_selection_combobox->currentText();
    m_ControlCoordinateSystemType = magic_enum::enum_cast<ControlCoordinateSystemType>(text.toStdString()).value();
}

