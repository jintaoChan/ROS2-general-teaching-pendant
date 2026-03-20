#include <QMimeData>
#include "task_detail.h"

TaskDetail::TaskDetail(QWidget *parent)
    : TreeViewWithKeyEvent(parent)
{
    m_RecycleTimeDelegate = new NumericItemDelegate(this);
    setItemDelegateForColumn(2, m_RecycleTimeDelegate);
    m_ItemFilterDelegate = new ItemFilterDelegate(this);
    setItemDelegateForColumn(0, m_ItemFilterDelegate);

    setDragEnabled(true);
    setAcceptDrops(true);
    setDropIndicatorShown(true);
    setDefaultDropAction(Qt::MoveAction);
    setDragDropMode(QAbstractItemView::InternalMove);
    // setDragDropMode(QAbstractItemView::DragDrop);
}

void TaskDetail::addPoint(const std::string& point_name)
{
    if(model() == nullptr) {
        QMessageBox::warning(this, "Task empty", "Select a task please!");
        return;
    }
    auto point_item = newStdString(point_name, false, true);
    setTaskActionType(point_item, TaskActionTypeEnum::Point);
    qobject_cast<QStandardItemModel *>(model())->appendRow({
        point_item,
        newStdString("Point"),
        newQString("")
    });
}

void TaskDetail::addGroup(const std::string &groupName, const int &recycleTimes)
{
    if(model() == nullptr) {
        QMessageBox::warning(this, "Task empty", "Select a task please!");
        return;
    }
    auto group_item = newStdString(groupName, true, true);
    setTaskActionType(group_item, TaskActionTypeEnum::Group);
    qobject_cast<QStandardItemModel *>(model())->appendRow({
        group_item,
        newStdString("Group"),
        newNumber(recycleTimes, true, false)
    });
}

void TaskDetail::addIO(const std::string& module_name, const std::string& interface_name, bool target_state)
{
    if(model() == nullptr) {
        QMessageBox::warning(this, "Task empty", "Select a task please!");
        return;
    }

    const auto state_text = target_state ? std::string("ON") : std::string("OFF");
    auto io_item = newStdString(module_name + "/" + interface_name + " -> " + state_text, false, true);
    setTaskActionType(io_item, TaskActionTypeEnum::IO);
    setTaskIOConfig(io_item, module_name, interface_name, target_state);

    qobject_cast<QStandardItemModel *>(model())->appendRow({
        io_item,
        newStdString("IO"),
        newQString("")
    });
}

void TaskDetail::deleteEvent(QModelIndex index)
{
    auto model = qobject_cast<QStandardItemModel *>(this->model());
    QModelIndex point_to_del_index = model->index(index.row(), 0);
    std::string point_to_del_name = model->data(point_to_del_index).toString().toStdString();
    model->removeRow(index.row());
}

void TaskDetail::TaskDeleted(QStandardItemModel *model)
{
    if(this->model() == model) {
        delete model;
        setModel(nullptr);
    }
}

void TaskDetail::dragEnterEvent(QDragEnterEvent *event) {
    event->acceptProposedAction();
}

void TaskDetail::dragMoveEvent(QDragMoveEvent *event) {
    event->acceptProposedAction();
    QTreeView::dragMoveEvent(event);
}

void TaskDetail::dropEvent(QDropEvent *event)
{
    if (event->mimeData()->hasText()) {
        QString text = event->mimeData()->text();
        addPoint(text.toStdString());
        event->acceptProposedAction();
    }
    else {
        QModelIndex index = indexAt(event->pos());
        if (index.isValid() && index.parent().isValid()) {
            event->ignore();
            return;
        }
        if (dropIndicatorPosition() == QAbstractItemView::OnItem && !isGroupRow(index.row())) {
            event->ignore();
            return;
        }
        QTreeView::dropEvent(event);
    }
}

bool TaskDetail::isGroupRow(int row)
{
    auto model = qobject_cast<QStandardItemModel *>(this->model());
    return getTaskActionType(model->item(row, 0)) == TaskActionTypeEnum::Group;
}
