#include <QtConcurrent/QtConcurrent>
#include <QInputDialog>
#include <QMessageBox>
#include "group_info_dialog.h"
#include "task_widget.h"
#include "ui_task_widget.h"
#include "robot_handle.h"
#include "task_executor.h"
#include "task_detail.h"
#include "move_task.h"
#include "group_task.h"
#include "io_task.h"

TaskWidget::TaskWidget(SettingPanel* setting_panel, QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::TaskWidget)
    , setting_panel_(setting_panel)
{
    ui->setupUi(this);
    connect(ui->task_list, &QTreeView::doubleClicked, this, &TaskWidget::handleTaskItemDoubleClicked);
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

void TaskWidget::handleTaskItemDoubleClicked(const QModelIndex &index) {
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

void TaskWidget::on_add_action_button_clicked()
{
    QStringList items;
    items << "Point" << "Group" << "IO";
    bool ok;
    QString selectedItem = QInputDialog::getItem(this, "Action Type", "Choose an action type", items, 0, false, &ok);
    if(!ok) {
        return;
    }

    if (selectedItem == "Point") {
        addPointAction();
    } else if (selectedItem == "Group") {
        addGroupAction();
    } else if (selectedItem == "IO") {
        addIOActionByDialog();
    }
}

void TaskWidget::addIOActionByDialog()
{
    std::vector<std::string> modules;
    const auto& output_modules = RobotHandle::instance().getIOOutputGroupsName();
    modules.insert(modules.end(), output_modules.begin(), output_modules.end());
    std::sort(modules.begin(), modules.end());
    modules.erase(std::unique(modules.begin(), modules.end()), modules.end());

    if (modules.empty()) {
        QMessageBox::warning(this, "IO", "No output IO module available.");
        return;
    }

    QStringList module_items;
    for (const auto& module : modules) {
        module_items << QString::fromStdString(module);
    }

    bool module_ok;
    QString selected_module = QInputDialog::getItem(this, "IO Module", "Choose module", module_items, 0, false, &module_ok);
    if(!module_ok) {
        return;
    }

    const auto& interfaces = RobotHandle::instance().getIOInterfacesName(selected_module.toStdString());
    if (interfaces.empty()) {
        QMessageBox::warning(this, "IO", "No IO interface in selected module.");
        return;
    }

    QStringList interface_items;
    for (const auto& interface_name : interfaces) {
        interface_items << QString::fromStdString(interface_name);
    }

    bool interface_ok;
    QString selected_interface = QInputDialog::getItem(this, "IO Interface", "Choose interface", interface_items, 0, false, &interface_ok);
    if(!interface_ok) {
        return;
    }

    QStringList target_items;
    target_items << "ON" << "OFF";
    bool state_ok;
    QString selected_state = QInputDialog::getItem(this, "IO Target", "Choose target state", target_items, 0, false, &state_ok);
    if(!state_ok) {
        return;
    }

    ui->task_detail->addIO(selected_module.toStdString(), selected_interface.toStdString(), selected_state == "ON");
}


void TaskWidget::addPointAction()
{
    QStringList items(ui->point_pool->getPointsName());
    bool ok;
    QString selectedItem = QInputDialog::getItem(nullptr, "Point selection", "Choose a point from point pool", items, 0, false, &ok);
    if(ok)
        ui->task_detail->addPoint(selectedItem.toStdString());
}

void TaskWidget::addGroupAction()
{
    GroupInfoDialog dialog;
    if (dialog.exec() == QDialog::Accepted) {
        auto newGroupName = dialog.getString();
        auto model = qobject_cast<QStandardItemModel *>(ui->task_detail->model());
        if(model != nullptr) {
            for(int i = 0; i < model->rowCount(); ++i) {
                if(model->item(i)->text() == newGroupName && getTaskActionType(model->item(i, 0)) == TaskActionTypeEnum::Group) {
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
        auto* action_item = editingTask->item(i, 0);
        auto name = action_item->text().toStdString();
        switch (getTaskActionType(action_item)) {
        case TaskActionTypeEnum::Group: {
            std::unique_ptr<GroupTask> group = std::make_unique<GroupTask>(name, editingTask->item(i, 2)->text().toInt());
            for(int j = 0; j < action_item->rowCount(); ++j) {
                auto* child_item = action_item->child(j, 0);
                if (child_item == nullptr) {
                    continue;
                }
                if (getTaskActionType(child_item) != TaskActionTypeEnum::Point) {
                    QMessageBox::warning(this, "Unsupported action", "Only point actions are supported inside groups currently.");
                    return;
                }
                auto point_name = child_item->text().toStdString();
                auto p = ui->point_pool->getPoint(point_name);
                std::unique_ptr<RobotTask> t = std::make_unique<MoveTask>(p);
                group->addAction(std::move(t));
            }
            TaskExecutor::instance().addTask(std::move(group));
            break;
        }
        case TaskActionTypeEnum::Point: {
            auto p = ui->point_pool->getPoint(name);
            std::unique_ptr<RobotTask> t = std::make_unique<MoveTask>(p);
            TaskExecutor::instance().addTask(std::move(t));
            break;
        }
        case TaskActionTypeEnum::IO:
        {
            const auto module_name = getTaskIOModule(action_item);
            const auto interface_name = getTaskIOInterface(action_item);
            if (module_name.empty() || interface_name.empty()) {
                QMessageBox::warning(this, "Invalid IO action", "IO action data is incomplete.");
                return;
            }
            const auto target_state = getTaskIOTargetState(action_item);
            std::unique_ptr<RobotTask> t = std::make_unique<IOTask>(module_name, interface_name, target_state);
            TaskExecutor::instance().addTask(std::move(t));
            break;
        }
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

