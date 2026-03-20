#pragma once

#include <QTreeView>
#include <QStandardItem>
#include <QKeyEvent>
#include "treeview_with_key_event.h"


enum class TaskActionTypeEnum : char {
    Point = 0,
    Group,
    IO
};

inline constexpr int kTaskActionTypeRole = Qt::UserRole + 1;
inline constexpr int kTaskIOConfigRole   = Qt::UserRole + 2;  // QVariantMap: {module, interface, target_state}

inline void setTaskActionType(QStandardItem* item, TaskActionTypeEnum type)
{
    if (item != nullptr) {
        item->setData(static_cast<int>(type), kTaskActionTypeRole);
    }
}

inline TaskActionTypeEnum getTaskActionType(const QStandardItem* item)
{
    if (item == nullptr) {
        return TaskActionTypeEnum::Point;
    }
    const auto type_data = item->data(kTaskActionTypeRole);
    if (!type_data.isValid()) {
        return TaskActionTypeEnum::Point;
    }
    return static_cast<TaskActionTypeEnum>(type_data.toInt());
}

inline void setTaskIOConfig(QStandardItem* item,
                            const std::string& module_name,
                            const std::string& interface_name,
                            bool target_state)
{
    if (item != nullptr) {
        QVariantMap config;
        config["module"]       = QString::fromStdString(module_name);
        config["interface"]    = QString::fromStdString(interface_name);
        config["target_state"] = target_state;
        item->setData(config, kTaskIOConfigRole);
    }
}

inline std::string getTaskIOModule(const QStandardItem* item)
{
    if (item == nullptr) return {};
    return item->data(kTaskIOConfigRole).toMap().value("module").toString().toStdString();
}

inline std::string getTaskIOInterface(const QStandardItem* item)
{
    if (item == nullptr) return {};
    return item->data(kTaskIOConfigRole).toMap().value("interface").toString().toStdString();
}

inline bool getTaskIOTargetState(const QStandardItem* item)
{
    if (item == nullptr) return false;
    return item->data(kTaskIOConfigRole).toMap().value("target_state").toBool();
}


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
    void addPoint(const std::string& point_name);
    void addGroup(const std::string& groupName, const int& recycleTimes);
    void addIO(const std::string& module_name, const std::string& interface_name, bool target_state);

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


