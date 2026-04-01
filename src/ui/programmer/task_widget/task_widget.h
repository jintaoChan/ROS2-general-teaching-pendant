#ifndef TASK_WIDGET_H
#define TASK_WIDGET_H

#include <QWidget>
#include <QStandardItem>
#include "robot_ports.h"
#include "setting_panel.h"
#include "task_executor.h"
#include "utils.h"


namespace Ui {
class TaskWidget;
}

class PointPool;

class TaskWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TaskWidget(SettingPanel* setting_panel, IRobotStateProvider* state_port, QWidget *parent = nullptr);
    ~TaskWidget();

    PointPool& pointPool();


public slots:
    void addPointFromControlPad(const TargetPointInfo& p);
private slots:
    void handleTaskItemDoubleClicked(const QModelIndex &index);
    void on_add_task_button_clicked();
    void on_add_action_button_clicked();
    void addPointAction();
    void addGroupAction();
    void on_execute_button_clicked();
    void on_delete_task_button_clicked();

    void on_stop_button_clicked();

private:
    void addIOActionByDialog();

private:
    Ui::TaskWidget *ui;
    SettingPanel* setting_panel_;
    IRobotStateProvider* state_port_{nullptr};
    TaskExecutor task_executor_;
};

#endif // TASK_WIDGET_H
