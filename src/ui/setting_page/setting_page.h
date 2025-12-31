#ifndef SETTING_PAGE_H
#define SETTING_PAGE_H

#include <QListWidget>
#include <QStackedWidget>
#include <QSplitter>
#include <QGridLayout>
#include <QWidget>

#include "setting_panel.h"

class SettingPage : public QWidget
{
    Q_OBJECT
public:
    explicit SettingPage(SettingPanel* setting_panel, QWidget *parent = nullptr);

signals:


private:
    SettingPanel* setting_panel_;
    QGridLayout* layout;
    QStackedWidget *stacked_widget;
    QListWidget* list_widget;
    QSplitter *splitter;
};

#endif // SETTING_PAGE_H
