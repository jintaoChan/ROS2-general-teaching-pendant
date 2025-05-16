#pragma once

#include <QTreeView>
#include <QStandardItem>
#include <QKeyEvent>
#include "treeview_with_key_event.h"

namespace Ui {
class TaskList;
}

class TaskList : public TreeViewWithKeyEvent
{
    Q_OBJECT

public:
    explicit TaskList(QWidget *parent = nullptr);
    ~TaskList(){};

public:
    void addTask(const std::string& taskName);
    void modifyTaskName(const std::string& oldTaskName, const std::string& newTaskName);
    void deleteTask(const std::string& taskName);
    QStandardItemModel* getTask(const std::string& taskName) const;

public:
    virtual void deleteEvent(QModelIndex index) override;

signals:
    void TaskDeleted(QStandardItemModel*);
    void ConfirmPointPoolToDeletedPoint(const std::string& pointName);

public slots:
    void CheckTaskListIfContainThisPoint(const std::string& pointName);

private:
    Ui::TaskList *ui;
    QStandardItemModel* m_Model;
    ItemFilterDelegate* m_ItemFilterDelegate;
    std::map<std::string, QStandardItemModel*> m_TaskLists;
};


