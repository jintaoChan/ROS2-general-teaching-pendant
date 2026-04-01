#ifndef POINT_POOL_H
#define POINT_POOL_H

#include <QTreeView>
#include <QStandardItem>
#include <QKeyEvent>
#include "treeview_with_key_event.h"
#include "robot_handle.h"
#include "utils.h"


class PointPool {
public:
    PointPool() = default;

public:
    void addPoint(const std::string& name, const TargetPointInfo& point) {
        point_pool_[name] = point;
    }

    void deletePoint(const std::string& name) {
        point_pool_.erase(name);
    }

    const TargetPointInfo& getPoint(const std::string& name) const {
        return point_pool_.at(name);
    }

    std::vector<std::string> getAllPointsName() const {
        std::vector<std::string> res;
        for(const auto& p: point_pool_) {
            res.push_back(p.first);
        }
        return res;
    }

private:
    MovePointInfo point_pool_;

};

namespace Ui {
class PointPoolWidget;
}

class PointPoolWidget : public TreeViewWithKeyEvent
{
    Q_OBJECT

public:
    explicit PointPoolWidget(QWidget *parent = nullptr);
    ~PointPoolWidget(){};

public:
    PointPool& pointPool() { return point_pool_; }
    const PointPool& pointPool() const { return point_pool_; }
    auto getPointsName() const -> QStringList;
    auto getPoint(const std::string& point_name) const -> TargetPointInfo;
    void addPoint(const TargetPointInfo& point_info);
    void deletePoint(QModelIndex index);
    void deletePoint(const std::string& point_name);

public:
    virtual void deleteEvent(QModelIndex index) override;

protected:
    void startDrag(Qt::DropActions supportedActions) override;
    QModelIndex addPoint(const std::string& point_name, const TargetPointInfo& point_info);

signals:
    void CallTaskListToCheckIfContainThisPoint(const std::string& name);

public slots:
    void modifyPointName(const std::string& old_name, const std::string& new_name);

private:
    PointPool point_pool_;
    Ui::PointPoolWidget *ui;
    ItemFilterDelegate* point_poolItem_filter_delegate_;
    QStandardItemModel* model_;
};



#endif // POINT_POOL_H
