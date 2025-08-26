#include <QTabWidget>
#include "main_window.h"
#include "ui_main_window.h"
#include "control_pad.h"
#include "task_widget.h"
#include "setting_panel.h"
#include "setting_page.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QWidget *central = new QWidget(this);
    setCentralWidget(central);

    QGridLayout* main_layout = new QGridLayout(ui->centralwidget);

    QTabWidget* tab_widget = new QTabWidget(ui->centralwidget);

    SettingPanel* setting_pannel = new SettingPanel(ui->centralwidget);
    ControlPad* control_pad = new ControlPad(setting_pannel, ui->centralwidget);
    tab_widget->addTab(control_pad, "Move");
    TaskWidget* task_widget = new TaskWidget(setting_pannel, ui->centralwidget);
    tab_widget->addTab(task_widget, "Task");
    SettingPage* setting_page = new SettingPage(ui->centralwidget);
    tab_widget->addTab(setting_page, "Setting");

    main_layout->addWidget(tab_widget);
    main_layout->addWidget(setting_pannel);

    central->setLayout(main_layout);

    connect(control_pad, &ControlPad::storeCurrentPointToPointPool, task_widget, &TaskWidget::addPointFromControlPad);
}

MainWindow::~MainWindow()
{
    delete ui;
}
