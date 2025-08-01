#include "main_window.h"
#include "ui_main_window.h"
#include "control_pad.h"
#include "task_widget.h"
#include "setting_panel.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QGridLayout* layout = new QGridLayout(this);
    SettingPanel* setting_pannel = new SettingPanel(this);
    ControlPad* controlPad = new ControlPad(setting_pannel, this);
    TaskWidget* taskWidget = new TaskWidget(setting_pannel, this);
    
    layout->addWidget(controlPad);
    layout->addWidget(setting_pannel);
    layout->addWidget(taskWidget);


    ui->centralwidget->setLayout(layout);

    connect(controlPad, &ControlPad::storeCurrentPointToPointPool, taskWidget, &TaskWidget::addPointFromControlPad);
}

MainWindow::~MainWindow()
{
    delete ui;
}
