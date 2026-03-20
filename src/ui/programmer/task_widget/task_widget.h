#ifndef TASK_WIDGET_H
#define TASK_WIDGET_H

#include <QWidget>
#include <QStandardItem>
#include "robot_handle.h"
#include "setting_panel.h"
#include "utils.h"


namespace Ui {
class TaskWidget;
}

class TaskWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TaskWidget(SettingPanel* setting_panel, QWidget *parent = nullptr);
    ~TaskWidget();


public slots:
    void addPointFromControlPad(const TargetPointInfo& p);
private slots:
    void on_task_item_clicked(const QModelIndex &index);
    void on_add_task_button_clicked();
    void on_add_action_button_clicked();
    void on_add_point_button_clicked();
    void on_add_group_button_clicked();
    void on_execute_button_clicked();
    void on_delete_task_button_clicked();

    void on_stop_button_clicked();

private:
    void addIOActionByDialog();

private:
    Ui::TaskWidget *ui;
    SettingPanel* setting_panel_;
};

#endif // TASK_WIDGET_H
