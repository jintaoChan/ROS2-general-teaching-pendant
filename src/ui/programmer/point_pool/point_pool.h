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
    auto getPoint(const std::string& pointName) const -> MovePointInfo;
    QModelIndex addPoint(const std::string& pointName, const MovePointInfo& moveGroupsState);
    void addPoint(const MovePointInfo& moveGroupsState);
    void deletePoint(QModelIndex index);
    void deletePoint(const std::string& pointName);

public:
    virtual void deleteEvent(QModelIndex index) override;

protected:
    void startDrag(Qt::DropActions supportedActions) override;

signals:
    void CallTaskListToCheckIfContainThisPoint(const std::string& name);

public slots:
    void modifyPointName(const std::string& oldName, const std::string& newName);

private:
    Ui::PointPool *ui;
    ItemFilterDelegate* m_PointPoolItemFilterDelegate;
    QStandardItemModel* m_Model;
    MovePointInfos m_PointPool;
};



#endif // POINT_POOL_H
