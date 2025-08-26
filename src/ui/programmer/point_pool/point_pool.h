#ifndef POINT_POOL_H
#define POINT_POOL_H

#include <QTreeView>
#include <QStandardItem>
#include <QKeyEvent>
#include "treeview_with_key_event.h"
#include "robot_handle.h"
#include "utils.h"

namespace Ui {
class PointPool;
}

class PointPool : public TreeViewWithKeyEvent
{
    Q_OBJECT

public:
    explicit PointPool(QWidget *parent = nullptr);
    ~PointPool(){};

public:
    auto getPointsName() const -> QStringList;
    auto getPoint(const std::string& point_name) const -> MovePointInfo;
    void addPoint(const MovePointInfo& move_groups_state);
    void deletePoint(QModelIndex index);
    void deletePoint(const std::string& point_name);

public:
    virtual void deleteEvent(QModelIndex index) override;

protected:
    void startDrag(Qt::DropActions supportedActions) override;
    QModelIndex addPoint(const std::string& point_name, const MovePointInfo& move_group_state);

signals:
    void CallTaskListToCheckIfContainThisPoint(const std::string& name);

public slots:
    void modifyPointName(const std::string& oldName, const std::string& new_name);

private:
    Ui::PointPool *ui;
    ItemFilterDelegate* point_poolItem_filter_delegate_;
    QStandardItemModel* model_;
    MovePointInfos point_pool_;
};



#endif // POINT_POOL_H
