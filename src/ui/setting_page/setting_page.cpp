#include <QLabel>
#include "setting_page.h"
#include "tcp.h"

SettingPage::SettingPage(QWidget *parent)
    : QWidget{parent}
{
    layout = new QGridLayout(this);
    list_widget = new QListWidget;
    list_widget->addItem("TCP");


    stacked_widget = new QStackedWidget;

    TCP* tcp = new TCP(this);
    stacked_widget->addWidget(tcp);

    QObject::connect(list_widget, &QListWidget::currentRowChanged,
                     stacked_widget, &QStackedWidget::setCurrentIndex);
    splitter = new QSplitter(this);
    splitter->addWidget(list_widget);
    splitter->addWidget(stacked_widget);

    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 4);
    layout->addWidget(splitter);

}
