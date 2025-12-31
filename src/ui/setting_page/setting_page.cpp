#include <QLabel>
#include <kdl/frames.hpp>
#include "setting_page.h"
#include "tcp.h"
#include "param_identification.h"

SettingPage::SettingPage(SettingPanel* setting_panel, QWidget *parent)
    :
    QWidget{parent},
    setting_panel_(setting_panel)
{
    layout = new QGridLayout(this);
    list_widget = new QListWidget;
    list_widget->addItem("TCP");
    list_widget->addItem("Param Identification");


    stacked_widget = new QStackedWidget;

    TCP* tcp = new TCP(this);
    ParamIdentification* iden = new ParamIdentification(this);
    stacked_widget->addWidget(tcp);
    stacked_widget->addWidget(iden);

    QObject::connect(list_widget, &QListWidget::currentRowChanged,
                     stacked_widget, &QStackedWidget::setCurrentIndex);
    splitter = new QSplitter(this);
    splitter->addWidget(list_widget);
    splitter->addWidget(stacked_widget);

    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 4);
    layout->addWidget(splitter);

    connect(iden, &ParamIdentification::identifyFinished, setting_panel_, &SettingPanel::activateDrag);
}
