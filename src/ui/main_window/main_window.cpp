#include "main_window.h"
#include "ui_main_window.h"
#include "control_pad.h"
#include "task_widget.h"
#include <QtConcurrent/QtConcurrent>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QGridLayout* layout = new QGridLayout(this);

    ControlPad* controlPad = new ControlPad(this);
    layout->addWidget(controlPad);
    QtConcurrent::run(controlPad, &ControlPad::run);

    TaskWidget* taskWidget = new TaskWidget(this);
    layout->addWidget(taskWidget);
    ui->centralwidget->setLayout(layout);

    connect(controlPad, &ControlPad::storeCurrentPointToPointPool, taskWidget, &TaskWidget::addPointFromControlPad);
}

MainWindow::~MainWindow()
{
    delete ui;
}
