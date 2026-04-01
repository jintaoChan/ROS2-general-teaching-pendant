#include <QTabWidget>
#include <QGridLayout>
#include "main_window.h"
#include "ui_main_window.h"
#include "control_pad.h"
#include "task_widget.h"
#include "setting_panel.h"
#include "setting_page.h"
#include "plot_page.h"
#include "io.h"


MainWindow::MainWindow(
    const AppPorts& ports,
    QWidget *parent)
    : QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QWidget* central = ui->centralwidget;
    QGridLayout* main_layout = qobject_cast<QGridLayout*>(central->layout());
    if(main_layout == nullptr) {
        main_layout = new QGridLayout(central);
    }

    QTabWidget* tab_widget = new QTabWidget(central);

    SettingPanel* setting_pannel = new SettingPanel(ports, central);
    ControlPad* control_pad = new ControlPad(setting_pannel, ports.state, central);
    tab_widget->addTab(control_pad, "Move");
    TaskWidget* task_widget = new TaskWidget(setting_pannel, ports.state, central);
    tab_widget->addTab(task_widget, "Task");
    SettingPage* setting_page = new SettingPage(setting_pannel, ports, task_widget->pointPool(), central);
    tab_widget->addTab(setting_page, "Setting");
    PlotPage* plot_page = new PlotPage(10000, ports.state, central);
    tab_widget->addTab(plot_page, "Plot");
    IOPanel* io_panel = new IOPanel(ports, central);
    tab_widget->addTab(io_panel, "IO");


    main_layout->addWidget(tab_widget, 0, 0);
    main_layout->addWidget(setting_pannel, 1, 0);
    main_layout->setRowStretch(0, 1);

    connect(control_pad, &ControlPad::storeCurrentPointToPointPool, task_widget, &TaskWidget::addPointFromControlPad);
}

MainWindow::~MainWindow()
{
    delete ui;
}
