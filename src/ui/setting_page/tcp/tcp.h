#pragma once

#include <QListWidget>
#include <QWidget>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QButtonGroup>
#include <QLabel>
#include <QLineEdit>
#include <QDialog>
#include <QDialogButtonBox>
#include <kdl/frames.hpp>
#include "robot_handle.h"
#include "tool_frame_widget.h"

class ListItemWidget : public QWidget {
    Q_OBJECT
public:
    explicit ListItemWidget(const QString &text, const int& index, QWidget *parent = nullptr)
        :
        QWidget(parent),
        index_(index)
    {
        label_ = new QLabel(text, this);
        edit_ = new QLineEdit(text, this);
        edit_->hide();
        edit_->setFrame(false);
        edit_->setStyleSheet("QLineEdit { border: none; background: transparent; }");
        check_box_ = new QCheckBox(this);

        QHBoxLayout *layout = new QHBoxLayout(this);
        layout->addWidget(label_);
        layout->addWidget(edit_);
        layout->addStretch();
        layout->addWidget(check_box_);
        setLayout(layout);

        connect(edit_, &QLineEdit::editingFinished, this, [this]() {
            QString new_name = edit_->text();
            QString old_name = label_->text();
            label_->setText(new_name);
            edit_->hide();
            label_->show();
            emit editFinished(new_name, old_name);
        });
    }

    void startEdit() {
        edit_->setText(label_->text());
        edit_->setReadOnly(false);
        label_->hide();
        edit_->show();
        edit_->setFocus();
        edit_->selectAll();
    }

public:
    QString text() const { return label_->text(); }
    void setText(const QString &t) { label_->setText(t); }

    bool isChecked() const { return check_box_->isChecked(); }
    void setChecked(bool c) { check_box_->setChecked(c); }

    QLineEdit* line_edit() const { return edit_; }
    QCheckBox* getCheckBox() const { return check_box_; }

    const int& getIndex() const { return index_; }

protected:
    void mouseDoubleClickEvent(QMouseEvent* event) override {
        if (label_->isVisible()) {
            label_->hide();
            edit_->show();
            edit_->setText(label_->text());
            edit_->setFocus();
            edit_->selectAll();
        }
        QWidget::mouseDoubleClickEvent(event);
    }

signals:
    void editFinished(const QString& new_name, const QString& old_name);

private:
    int index_;
    QLabel* label_;
    QLineEdit* edit_;
    QCheckBox* check_box_;
};

class StringSelectionDialog : public QDialog {
    Q_OBJECT
public:
    StringSelectionDialog(const std::vector<std::string>& items, QWidget *parent = nullptr)
        : QDialog(parent)
    {
        setWindowTitle("Please select points to calibrate");

        auto *layout = new QVBoxLayout(this);

        list_widget = new QListWidget(this);
        list_widget->setSelectionMode(QAbstractItemView::NoSelection);
        list_widget->setAlternatingRowColors(true);

        for (const auto &s : items) {
            QListWidgetItem *item = new QListWidgetItem(QString::fromStdString(s), list_widget);
            item->setCheckState(Qt::Unchecked);
        }

        layout->addWidget(list_widget);

        QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
        layout->addWidget(buttonBox);
    }

    std::vector<std::string> selectedItems() const {
        std::vector<std::string> result;
        for (int i = 0; i < list_widget->count(); ++i) {
            QListWidgetItem *item = list_widget->item(i);
            if (item->checkState() == Qt::Checked) {
                result.push_back(item->text().toStdString());
            }
        }
        return result;
    }

private:
    QListWidget *list_widget;
};

namespace Ui {
class TCP;
}

class TCP : public QWidget
{
    Q_OBJECT
public:
    explicit TCP(QWidget *parent = nullptr);
    ~TCP();

    bool eventFilter(QObject* obj, QEvent* event);
signals:

private slots:
    void on_add_new_button_clicked();

    void on_calibrate_button_clicked();

private:
    ListItemWidget* addNewToolFrame(const QString& name, const KDL::Frame& frame = KDL::Frame{});
    QListWidgetItem* findItemByWidget(QListWidget* list, QWidget* w);

private:
    Ui::TCP *ui_;
    const ToolInfo& tool_info_;
    QButtonGroup* check_box_group_;
    ToolFrameWidget* tool_frame_widget_;
    ListItemWidget* current_selected_tool{nullptr};
    std::string current_tool_name_;
};



