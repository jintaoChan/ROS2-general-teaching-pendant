#include "task_list.h"

TaskList::TaskList(QWidget *parent)
    : TreeViewWithKeyEvent(parent)
{
    m_Model = new QStandardItemModel;
    m_Model->setHorizontalHeaderLabels(QStringList() << "Name");
    setModel(m_Model);
    m_ItemFilterDelegate = new ItemFilterDelegate(this);
    setItemDelegate(m_ItemFilterDelegate);
    connect(m_ItemFilterDelegate, &ItemFilterDelegate::itemAdded, this, &TaskList::addTask);
    connect(m_ItemFilterDelegate, &ItemFilterDelegate::itemModified, this, &TaskList::modifyTaskName);
}


void TaskList::addTask(const std::string &taskName)
{
    QStandardItemModel* model = new QStandardItemModel(this);
    model->setHorizontalHeaderLabels(QStringList()
                                     << "Point/Group Name"
                                     << "Recycle Times"
                                     );
    m_TaskLists[taskName] = model;
}

void TaskList::modifyTaskName(const std::string &oldTaskName, const std::string &newTaskName)
{
    m_TaskLists[newTaskName] = m_TaskLists[oldTaskName];
    m_TaskLists.erase(oldTaskName);
}

void TaskList::deleteTask(const std::string &taskName)
{
    for(int i = 0;i < m_Model->rowCount(); ++i) {
        if(m_Model->item(i)->text().toStdString() == taskName) {
            m_TaskLists.erase(taskName);
            m_Model->removeRow(i);
            break;
        }
    }
}

QStandardItemModel *TaskList::getTask(const std::string &taskName) const
{
    return m_TaskLists.at(taskName);
}

void TaskList::deleteEvent(QModelIndex index)
{
    auto name = m_Model->index(index.row(), index.column()).data().toString().toStdString();
    emit(TaskDeleted(m_TaskLists[name]));
    deleteTask(name);
}

void TaskList::CheckTaskListIfContainThisPoint(const std::string& pointName)
{
    QStringList involvedTasks;
    for(const auto& task: m_TaskLists) {
        bool found = findChildrenWithText(task.second->invisibleRootItem(), QString::fromStdString(pointName));
        if(found){
            involvedTasks.push_back(QString::fromStdString(task.first));
        }
    }
    if(!involvedTasks.empty()) {
        QMessageBox::StandardButton reply = QMessageBox::question(
            this,
            "Warning",
            QString("Tasks: \"%1\" exist point \"%2\" . Do you want to delete it along with the one in above tasks?").arg(involvedTasks.join(", "), QString::fromStdString(pointName)),
            QMessageBox::Yes | QMessageBox::No
        );
        bool allowDelete = (reply == QMessageBox::Yes);
        if(!allowDelete) { return; }

        for(const auto& task: m_TaskLists) {
            deleteChildrenWithText(task.second->invisibleRootItem(), QString::fromStdString(pointName));
        }
    }

    emit(ConfirmPointPoolToDeletedPoint(pointName));
}

