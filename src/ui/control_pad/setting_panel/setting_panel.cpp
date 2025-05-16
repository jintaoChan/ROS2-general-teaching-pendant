#include <QDoubleValidator>
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

