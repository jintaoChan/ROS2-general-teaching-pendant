#ifndef SETTING_PAGE_H
#define SETTING_PAGE_H

#include <QListWidget>
#include <QStackedWidget>
#include <QSplitter>
#include <QGridLayout>
#include <QWidget>

class SettingPage : public QWidget
{
    Q_OBJECT
public:
    explicit SettingPage(QWidget *parent = nullptr);

signals:


private:
    QGridLayout* layout;
    QStackedWidget *stacked_widget;
    QListWidget* list_widget;
    QSplitter *splitter;
};

#endif // SETTING_PAGE_H
