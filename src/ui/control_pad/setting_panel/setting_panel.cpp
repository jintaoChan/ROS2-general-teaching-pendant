#include <QDoubleValidator>
#include <magic_enum/magic_enum.hpp>
#include "setting_panel.h"
#include "ui_setting_panel.h"

SettingPanel::SettingPanel(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::SettingPanel)
{
    ui->setupUi(this);
    QDoubleValidator * validator = new QDoubleValidator(this);
    ui->lineEdit_step->setValidator(validator);
    ui->lineEdit_velocity->setValidator(validator);
    constexpr auto coordEntries = magic_enum::enum_entries<ControlCoordinateSystemType>();
    for(const auto& coord : coordEntries) {
        ui->coordinate_selection_combobox->addItem(QString::fromStdString(std::string(coord.second)));
    }
    ui->coordinate_selection_combobox->setCurrentText(QString::fromStdString(std::string(magic_enum::enum_name(ControlCoordinateSystemType::TOOL))));
}

SettingPanel::~SettingPanel()
{
    delete ui;
}

double SettingPanel::getStep()
{
    return ui->lineEdit_step->text().toDouble();
}

double SettingPanel::getVelocity()
{
    return ui->lineEdit_velocity->text().toDouble();
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

