#include "treeview_with_key_event.h"

TreeViewWithKeyEvent::TreeViewWithKeyEvent(QWidget *parent)
    : QTreeView(parent)
{
    
}


void TreeViewWithKeyEvent::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Delete) {
        QModelIndex current = currentIndex();
        if (!current.isValid())
            return;
        if (!current.parent().isValid()) {
            deleteEvent(current);
        }
    } else {
        QTreeView::keyPressEvent(event);
    }
}
