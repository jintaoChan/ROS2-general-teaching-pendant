#include <QtConcurrent/QtConcurrent>
#include <QInputDialog>
#include <QMessageBox>
#include "group_info_dialog.h"
#include "task_widget.h"
#include "ui_task_widget.h"
#include "robot_handle.h"
#include "task_executor.h"
#include "move_task.h"
#include "group_task.h"

TaskWidget::TaskWidget(SettingPanel* setting_panel, QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::TaskWidget)
    , setting_panel_(setting_panel)
{
    ui->setupUi(this);
    connect(ui->task_list, &QTreeView::doubleClicked, this, &TaskWidget::on_task_item_clicked);
    connect(ui->task_list, &TaskList::TaskDeleted, ui->task_detail, &TaskDetail::TaskDeleted);
    connect(ui->point_pool, &PointPoolWidget::CallTaskListToCheckIfContainThisPoint, ui->task_list, &TaskList::CheckTaskListIfContainThisPoint);
    connect(ui->task_list, &TaskList::ConfirmPointPoolToDeletedPoint, ui->point_pool, static_cast<void(PointPoolWidget::*)(const std::string&)>(&PointPoolWidget::deletePoint));

    TaskExecutor::instance().setStateCallback([this](ExecutorState state) {
        QMetaObject::invokeMethod(this, [this, state]() {
            if (state == ExecutorState::IDLE) {
                this->ui->execute_button->setEnabled(true);
                this->ui->stop_button->setEnabled(false);
            } else if (state == ExecutorState::RUNNING) {
                this->ui->execute_button->setEnabled(false);
                this->ui->stop_button->setEnabled(true);
            }
        }, Qt::QueuedConnection);
    });
}

TaskWidget::~TaskWidget()
{
    delete ui;
}

void TaskWidget::addPointFromControlPad(const TargetPointInfo &p)
{
    ui->point_pool->addPoint(p);
}

void TaskWidget::on_task_item_clicked(const QModelIndex &index) {
    auto model = qobject_cast<QStandardItemModel *>(ui->task_list->model());
    QStandardItem *item = model->itemFromIndex(index);
    if (item->column() == 0) {
        ui->task_detail->setModel(ui->task_list->getTask(item->text().toStdString()));
        ui->editing_task_label->setText(item->text());
    }
}

void TaskWidget::on_add_task_button_clicked()
{
    QStandardItem *item = new QStandardItem("");
    auto model = qobject_cast<QStandardItemModel *>(ui->task_list->model());
    model->appendRow(item);
    QModelIndex index = model->indexFromItem(item);
    ui->task_list->edit(index);
}


void TaskWidget::on_add_point_button_clicked()
{
    QStringList items(ui->point_pool->getPointsName());
    bool ok;
    QString selectedItem = QInputDialog::getItem(nullptr, "Point selection", "Choose a point from point pool", items, 0, false, &ok);
    if(ok)
        ui->task_detail->addPoint(selectedItem.toStdString());
}

void TaskWidget::on_add_group_button_clicked()
{
    GroupInfoDialog dialog;
    if (dialog.exec() == QDialog::Accepted) {
        auto newGroupName = dialog.getString();
        auto model = qobject_cast<QStandardItemModel *>(ui->task_detail->model());
        if(model != nullptr) {
            for(int i = 0; i < model->rowCount(); ++i) {
                if(model->item(i)->text() == newGroupName && model->item(i, 1) != nullptr && !model->item(i, 1)->text().isEmpty()) {
                    QMessageBox::warning(this, "Warning", "Name Repetition with other action");
                    return;
                }
            }
        }
        if(ui->point_pool->getPointsName().contains(newGroupName)) {
            QMessageBox::warning(this, "Warning", "Name Repetition with point in point pool");
            return;
        }
        ui->task_detail->addGroup(newGroupName.toStdString(), dialog.getInteger());
    }
}

void TaskWidget::on_execute_button_clicked()
{
    auto editingTask = qobject_cast<QStandardItemModel *>(ui->task_detail->model());
    if(editingTask == nullptr) {
        QMessageBox::warning(this, "Task empty", "Select a task please!");
        return;
    }
    for(int i = 0; i < editingTask->rowCount(); ++i) {
        auto name = editingTask->item(i)->text().toStdString();
        if (editingTask->item(i, 1) != nullptr && !editingTask->item(i, 1)->text().isEmpty()) {
            std::unique_ptr<GroupTask> group = std::make_unique<GroupTask>(name, editingTask->item(i, 1)->text().toInt());
            for(int j = 0; j < editingTask->item(i)->rowCount(); ++j) {
                auto point_name = editingTask->item(i)->child(j)->text().toStdString();
                auto p = ui->point_pool->getPoint(point_name);
                std::unique_ptr<RobotTask> t = std::make_unique<MoveTask>(p);
                group->addAction(std::move(t));
            }
            TaskExecutor::instance().addTask(std::move(group));
        }
        else {
            auto p = ui->point_pool->getPoint(name);
            std::unique_ptr<RobotTask> t = std::make_unique<MoveTask>(p);
            TaskExecutor::instance().addTask(std::move(t));
        }
    }
    TaskExecutor::instance().start();
}


void TaskWidget::on_delete_task_button_clicked()
{
    auto selecteds = ui->task_list->selectionModel()->selectedIndexes();
    std::for_each(selecteds.begin(), selecteds.end(), [&](const auto& s){ui->task_list->deleteEvent(s);});
}


void TaskWidget::on_stop_button_clicked()
{
    TaskExecutor::instance().stop();
}

