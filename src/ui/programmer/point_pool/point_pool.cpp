#include <QMimeData>
#include <QDrag>
#include "point_pool.h"

PointPool::PointPool(QWidget *parent)
    : TreeViewWithKeyEvent(parent)
{
    m_Model = new QStandardItemModel;
    m_Model->setHorizontalHeaderLabels(QStringList()
                                        << "Name"
                                        << "Target Type/Values"
                                     );
    m_PointPoolItemFilterDelegate = new ItemFilterDelegate(this);
    this->setItemDelegate(m_PointPoolItemFilterDelegate);
    connect(m_PointPoolItemFilterDelegate,
            &ItemFilterDelegate::itemAdded,
            m_PointPoolItemFilterDelegate,
            [this](const std::string& newName){emit m_PointPoolItemFilterDelegate->itemModified("", newName);});
    connect(m_PointPoolItemFilterDelegate, &ItemFilterDelegate::itemModified, this, &PointPool::modifyPointName);
    auto& robotDes = RobotDescription::instance();
    MovePointInfos presetPoints = robotDes.getPresetGroupState();
    for(const auto& point : presetPoints){
        addPoint(point.first, point.second);
    }
    setModel(m_Model);
    setDragEnabled(true);
    // setDragDropMode(QAbstractItemView::InternalMove);//drag disable
    // expandAll();
    show();
}

QStringList PointPool::getPointsName() const
{
    QStringList res;
    for(int i = 0 ;i < m_Model->rowCount(); ++i) {
        res << m_Model->item(i)->text();
    }
    return res;
}

MovePointInfo PointPool::getPoint(const std::string &pointName) const
{
    return m_PointPool.at(pointName);
}

QModelIndex PointPool::addPoint(const std::string &pointName, const MovePointInfo &moveGroupsState)
{
    QStandardItem* pointItem = newStdString(pointName, true);
    for(const auto& group : moveGroupsState) {
        QStandardItem* groupItem = newStdString(group.first);
        switch (group.second.MoveType) {
        case MoveTypeEnum::JOINT:
            groupItem->appendColumn({});
            for(size_t i = 0 ; i < group.second.JointNames.size(); ++i) {
                groupItem->appendRow({
                    newStdString(group.second.JointNames[i]),
                    newNumber(group.second.Values[i]),
                });
            }
            break;
        default:
            break;
        }
        // pointItem->appendRow({groupItem, newNumber((uchar)group.second.MoveType)});
        pointItem->appendRow({groupItem, newStdString("JOINT")});
    }
    m_Model->appendRow({pointItem, newQString("")});
    m_PointPool[pointName] = moveGroupsState;
    return m_Model->indexFromItem(pointItem);
}

void PointPool::addPoint(const MovePointInfo &moveGroupsState)
{
    auto index = addPoint("", moveGroupsState);
    edit(index);
}

void PointPool::deletePoint(QModelIndex index)
{
    QModelIndex pointToDelIndex = m_Model->index(index.row(), 0);
    std::string pointToDelName = m_Model->data(pointToDelIndex).toString().toStdString();
    m_PointPool.erase(pointToDelName);
    m_Model->removeRow(index.row());
}

void PointPool::deletePoint(const std::string &pointName)
{
    for(int i = 0;i < m_Model->rowCount(); ++i) {
        if(m_Model->item(i)->text().toStdString() == pointName) {
            m_PointPool.erase(pointName);
            m_Model->removeRow(i);
            break;
        }
    }
}

void PointPool::deleteEvent(QModelIndex index)
{
    QModelIndex pointToDelIndex = m_Model->index(index.row(), 0);
    std::string pointToDelName = m_Model->data(pointToDelIndex).toString().toStdString();
    emit(CallTaskListToCheckIfContainThisPoint(pointToDelName));
    // deletePoint(index);
}

void PointPool::startDrag(Qt::DropActions supportedActions)
{
    QModelIndex index = currentIndex();
    if (!index.isValid()) return;
    if (index.parent().isValid()) return;
    if (index.column() != 0) return;

    QString text = model()->data(index, Qt::DisplayRole).toString();
    QMimeData *mimeData = new QMimeData;
    mimeData->setText(text);

    QDrag *drag = new QDrag(this);
    drag->setMimeData(mimeData);
    drag->exec(Qt::CopyAction);
}

void PointPool::modifyPointName(const std::string &oldName, const std::string &newName)
{
    m_PointPool[newName] = m_PointPool[oldName];
    m_PointPool.erase(oldName);
}
