#include <QMimeData>
#include <QDrag>
#include <QInputDialog>
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
    auto& robotDes = RobotHandle::instance();
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

void PointPool::addPoint(const MovePointInfo &moveGroupsState)
{
    bool ok;
    QString text = QInputDialog::getText(this, tr("Add a point"),
                                         tr("Please input point's name"), QLineEdit::Normal,
                                         "", &ok);
    auto index = addPoint("", moveGroupsState);
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
    m_Model->appendRow({point_item, newQString("")});
    m_PointPool[point_name] = move_group_state;
    return m_Model->indexFromItem(point_item);
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
