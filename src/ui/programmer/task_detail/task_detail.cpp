#include <QMimeData>
#include "task_detail.h"

TaskDetail::TaskDetail(QWidget *parent)
    : TreeViewWithKeyEvent(parent)
{
    m_RecycleTimeDelegate = new NumericItemDelegate(this);
    setItemDelegateForColumn(1, m_RecycleTimeDelegate);
    m_ItemFilterDelegate = new ItemFilterDelegate(this);
    setItemDelegateForColumn(0, m_ItemFilterDelegate);

    setDragEnabled(true);
    setAcceptDrops(true);
    setDropIndicatorShown(true);
    setDefaultDropAction(Qt::MoveAction);
    setDragDropMode(QAbstractItemView::InternalMove);
    // setDragDropMode(QAbstractItemView::DragDrop);
}

void TaskDetail::addPoint(const std::string& pointName)
{
    if(model() == nullptr) {
        QMessageBox::warning(this, "Task empty", "Select a task please!");
        return;
    }
    qobject_cast<QStandardItemModel *>(model())->appendRow(newStdString(pointName, false, true));
}

void TaskDetail::addGroup(const std::string &groupName, const int &recycleTimes)
{
    if(model() == nullptr) {
        QMessageBox::warning(this, "Task empty", "Select a task please!");
        return;
    }
    qobject_cast<QStandardItemModel *>(model())->appendRow({newStdString(groupName, true, true), newNumber(recycleTimes, true, false)});
}

void TaskDetail::deleteEvent(QModelIndex index)
{
    auto model = qobject_cast<QStandardItemModel *>(this->model());
    QModelIndex pointToDelIndex = model->index(index.row(), 0);
    std::string pointToDelName = model->data(pointToDelIndex).toString().toStdString();
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
    return model->item(row, 1) != nullptr && !model->item(row, 1)->text().isEmpty();
}
