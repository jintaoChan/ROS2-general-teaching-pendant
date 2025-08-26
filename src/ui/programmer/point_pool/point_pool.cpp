#include <QMimeData>
#include <QDrag>
#include <QInputDialog>
#include "point_pool.h"

PointPool::PointPool(QWidget *parent)
    : TreeViewWithKeyEvent(parent)
{
    model_ = new QStandardItemModel;
    model_->setHorizontalHeaderLabels(QStringList()
                                        << "Name"
                                        << "Target Type/Values"
                                     );
    point_poolItem_filter_delegate_ = new ItemFilterDelegate(this);
    this->setItemDelegate(point_poolItem_filter_delegate_);
    connect(point_poolItem_filter_delegate_,
            &ItemFilterDelegate::itemAdded,
            point_poolItem_filter_delegate_,
            [this](const std::string& new_name){emit point_poolItem_filter_delegate_->itemModified("", new_name);});
    connect(point_poolItem_filter_delegate_, &ItemFilterDelegate::itemModified, this, &PointPool::modifyPointName);
    auto& robot_des = RobotHandle::instance();
    MovePointInfos preset_points = robot_des.getPresetGroupState();
    for(const auto& point : preset_points){
        addPoint(point.first, point.second);
    }
    setModel(model_);
    setDragEnabled(true);
    // setDragDropMode(QAbstractItemView::InternalMove);//drag disable
    // expandAll();
    show();
}

QStringList PointPool::getPointsName() const
{
    QStringList res;
    for(int i = 0 ;i < model_->rowCount(); ++i) {
        res << model_->item(i)->text();
    }
    return res;
}

MovePointInfo PointPool::getPoint(const std::string &point_name) const
{
    return point_pool_.at(point_name);
}

void PointPool::addPoint(const MovePointInfo &move_groups_state)
{
    bool ok;
    QString text = QInputDialog::getText(this, tr("Add a point"),
                                         tr("Please input point's name"), QLineEdit::Normal,
                                         "", &ok);
    auto index = addPoint("", move_groups_state);
    QWidget* editor = itemDelegate(index)->createEditor(this, QStyleOptionViewItem(), index);

    QLineEdit* line = qobject_cast<QLineEdit*>(editor);
    if (line) {
        line->setText(text);
        itemDelegate(index)->setModelData(editor, model(), index);
    }
    delete editor;
}

void PointPool::deletePoint(QModelIndex index)
{
    QModelIndex point_to_del_index = model_->index(index.row(), 0);
    std::string point_to_del_name = model_->data(point_to_del_index).toString().toStdString();
    point_pool_.erase(point_to_del_name);
    model_->removeRow(index.row());
}

void PointPool::deletePoint(const std::string &point_name)
{
    for(int i = 0;i < model_->rowCount(); ++i) {
        if(model_->item(i)->text().toStdString() == point_name) {
            point_pool_.erase(point_name);
            model_->removeRow(i);
            break;
        }
    }
}

void PointPool::deleteEvent(QModelIndex index)
{
    QModelIndex point_to_del_index = model_->index(index.row(), 0);
    std::string point_to_del_name = model_->data(point_to_del_index).toString().toStdString();
    emit(CallTaskListToCheckIfContainThisPoint(point_to_del_name));
    // deletePoint(index);
}

QModelIndex PointPool::addPoint(const std::string &point_name, const MovePointInfo &move_group_state)
{
    QStandardItem* point_item = newStdString(point_name, true);
    for(const auto& group : move_group_state) {
        QStandardItem* group_item = newStdString(group.first);
        switch (group.second.MoveType) {
        case MoveTypeEnum::JOINT:
            group_item->appendColumn({});
            for(size_t i = 0 ; i < group.second.JointNames.size(); ++i) {
                group_item->appendRow({
                    newStdString(group.second.JointNames[i]),
                    newNumber(group.second.Values[i]),
                });
            }
            break;
        default:
            break;
        }
        point_item->appendRow({group_item, newStdString("JOINT")});
    }
    model_->appendRow({point_item, newQString("")});
    point_pool_[point_name] = move_group_state;
    return model_->indexFromItem(point_item);
}

void PointPool::startDrag(Qt::DropActions)
{
    QModelIndex index = currentIndex();
    if (!index.isValid()) return;
    if (index.parent().isValid()) return;
    if (index.column() != 0) return;

    QString text = model()->data(index, Qt::DisplayRole).toString();
    QMimeData *mime_data = new QMimeData;
    mime_data->setText(text);

    QDrag *drag = new QDrag(this);
    drag->setMimeData(mime_data);
    drag->exec(Qt::CopyAction);
}

void PointPool::modifyPointName(const std::string &oldName, const std::string &new_name)
{
    point_pool_[new_name] = point_pool_[oldName];
    point_pool_.erase(oldName);
}
