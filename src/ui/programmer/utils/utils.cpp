#include "utils.h"

bool findChildrenWithText(QStandardItem *current, const QString &targetText) {
    if (!current) return false;
    if (current->text() == targetText)
        return true;
    for (int i = 0; i < current->rowCount(); ++i) {
        QStandardItem *child = current->child(i);
        if (findChildrenWithText(child, targetText))
            return true;
    }

    return false;
}

void deleteChildrenWithText(QStandardItem *current, const QString &targetText)
{
    for (int row = current->rowCount() - 1; row >= 0; --row) {
        QStandardItem *child = current->child(row);
        if(child) {
            deleteChildrenWithText(child, targetText);
            if (child->text() == targetText) {
                current->removeRow(row);
            }
        }
    }
}

QStandardItem *newQString(const QString &s, bool editable, bool dragable)
{
    auto item = new QStandardItem(s);
    if (editable) {
        item->setFlags(item->flags() | Qt::ItemIsEditable);
    } else {
        item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    }

    if (dragable) {
        item->setFlags(item->flags() | Qt::ItemIsDropEnabled);
    } else {
        item->setFlags(item->flags() & ~Qt::ItemIsDropEnabled);
    }

    return item;
}

QStandardItem *newStdString(const std::string& s, bool editable, bool dragable)
{
    return newQString(QString::fromStdString(s), editable, dragable);

}

