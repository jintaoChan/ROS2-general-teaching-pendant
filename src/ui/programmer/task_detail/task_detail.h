#pragma once

#include <QTreeView>
#include <QStandardItem>
#include <QKeyEvent>
#include "treeview_with_key_event.h"


enum class TaskTargetTypeEnum : char {
    Point = 0,
    Group
};

namespace Ui {
class TaskDetail;
}

class TaskDetail : public TreeViewWithKeyEvent
{
    Q_OBJECT

public:
    explicit TaskDetail(QWidget *parent = nullptr);
    ~TaskDetail(){};

public:
    void addPoint(const std::string& pointName);
    void addGroup(const std::string& groupName, const int& recycleTimes);

    virtual void deleteEvent(QModelIndex index) override;

public slots:
    void TaskDeleted(QStandardItemModel* model);

protected:
    void dragEnterEvent(QDragEnterEvent *event) override;
    void dragMoveEvent(QDragMoveEvent *event) override;
    void dropEvent(QDropEvent *event) override;

private:
    bool isGroupRow(int row);

private:
    Ui::TaskDetail *ui;
    NumericItemDelegate* m_RecycleTimeDelegate;
    ItemFilterDelegate* m_ItemFilterDelegate;

};


