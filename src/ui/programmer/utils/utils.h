#ifndef UTILS_H
#define UTILS_H

#include <QStandardItem>
#include <QStyledItemDelegate>
#include <QMessageBox>
#include <QLineEdit>

class ItemFilterDelegate : public QStyledItemDelegate {
    Q_OBJECT
public:
    using QStyledItemDelegate::QStyledItemDelegate;

    void setModelData(QWidget *editor, QAbstractItemModel *model,
                      const QModelIndex &index) const override {
        QLineEdit *line_edit = qobject_cast<QLineEdit *>(editor);
        QString new_text = line_edit->text().trimmed();
        QString old_text = index.data().toString().trimmed();

        if (line_edit) {
            if (new_text.isEmpty()) {
                QMessageBox::warning(editor, "Warning", "Empty name is not allowed");
                if(old_text.isEmpty()){
                    model->removeRow(index.row());
                }
                return;
            }
        }
        int row_count = model->rowCount(index.parent());
        for (int i = 0; i < row_count; ++i) {
            if (i == index.row()) continue;
            QModelIndex sibling = model->index(i, index.column(), index.parent());
            if (sibling.data().toString() == new_text) {
                model->removeRow(index.row(), index.parent());
                QMessageBox::warning(editor, "Warning", "Name Repetition");
                return;
            }
        }
        if (old_text.isEmpty()) {
            emit itemAdded(new_text.toStdString());
        } else if (old_text != new_text) {
            emit itemModified(old_text.toStdString(), new_text.toStdString());
        }
        QStyledItemDelegate::setModelData(editor, model, index);
    }

signals:
    void itemAdded(const std::string& s) const;
    void itemModified(const std::string& src, const std::string& dst) const;
};

class NumericItemDelegate : public QStyledItemDelegate {
    Q_OBJECT

public:
    using QStyledItemDelegate::QStyledItemDelegate;

    QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &,
                          const QModelIndex &) const override {
        QLineEdit *editor = new QLineEdit(parent);
        auto validator = new QIntValidator(editor);
        validator->setRange(-1, std::numeric_limits<int>::max());
        editor->setValidator(validator);
        return editor;
    }

    void setEditorData(QWidget *editor, const QModelIndex &index) const override {
        QString value = index.model()->data(index, Qt::EditRole).toString();
        QLineEdit *line_edit = static_cast<QLineEdit *>(editor);
        line_edit->setText(value);
    }

    void setModelData(QWidget *editor, QAbstractItemModel *model,
                      const QModelIndex &index) const override {
        QLineEdit *line_edit = static_cast<QLineEdit *>(editor);
        model->setData(index, line_edit->text(), Qt::EditRole);
    }
};


bool findChildrenWithText(QStandardItem *current, const QString &targetText);
void deleteChildrenWithText(QStandardItem *current, const QString &targetText);

//helper function
QStandardItem* newStdString(const std::string& s, bool editable = false, bool dragable = false);
QStandardItem* newQString(const QString& s, bool editable = false, bool dragable = false);
template <typename T>
QStandardItem* newNumber(T number, bool editable = false, bool dragable = false){
    return newQString(QString::number(number), editable, dragable);
}


#endif // UTILS_H
