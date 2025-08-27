#include <QStackedWidget>
#include <QLabel>
#include <QSplitter>
#include <QGridLayout>
#include <QMessageBox>
#include <QKeyEvent>
#include <QInputDialog>
#include "kinematics_plugin.h"
#include "tcp.h"
#include "ui_tcp.h"
#include "point_pool.h"

TCP::TCP(QWidget *parent)
    : QWidget{parent},
    ui_(new Ui::TCP),
    tool_info_(RobotHandle::instance().getRobotArmToolInfo())
{
    ui_->setupUi(this);
    ui_->tcp_list->installEventFilter(this);

    tool_frame_widget_ = new ToolFrameWidget(this);
    ui_->layout->addWidget(tool_frame_widget_);


    connect(ui_->tcp_list, &QListWidget::itemClicked, this, [this](QListWidgetItem *item) {
        auto widget = static_cast<ListItemWidget*>(ui_->tcp_list->itemWidget(item));
        current_selected_tool = widget;
        auto tool_info = tool_info_.at(widget->text().toStdString());
        double rx, ry, rz;
        tool_info.M.GetEulerZYX(rz, ry, rx);
        tool_frame_widget_->setValue(tool_info.p.x(), tool_info.p.y(), tool_info.p.z(), rx, ry, rz);
    });

    connect(tool_frame_widget_, &ToolFrameWidget::toolInfoModified, this,
        [this](const auto& x, const auto& y, const auto& z, const auto& rx, const auto& ry, const auto& rz){
        auto frame = KDL::Frame(KDL::Rotation::EulerZYX(rz, ry, rx), KDL::Vector(x, y, z));
        auto selected_tool_name = current_selected_tool->text().toStdString();
        RobotHandle::instance().deleteToolFrame(selected_tool_name);
        RobotHandle::instance().addToolFrame(selected_tool_name, frame);
        KinematicsPlugin::instance().refreshCoordinateSystem();
    });


    check_box_group_ = new QButtonGroup(this);
    check_box_group_->setExclusive(true);
    current_tool_name_ = RobotHandle::instance().getCurrentToolFrame();
    for(const auto& t : tool_info_) {
        addNewToolFrame(QString::fromStdString(t.first));
    }

}

TCP::~TCP()
{
    delete ui_;
}

bool TCP::eventFilter(QObject* obj, QEvent* event)
{
    if (obj == ui_->tcp_list && event->type() == QEvent::KeyPress) {
        auto keyEvent = static_cast<QKeyEvent*>(event);
        if (keyEvent->key() == Qt::Key_Delete) {
            auto item = ui_->tcp_list->currentItem();
            if (item) {
                auto widget = (ListItemWidget*)ui_->tcp_list->itemWidget(item);
                RobotHandle::instance().deleteToolFrame(widget->text().toStdString());
                delete ui_->tcp_list->takeItem(ui_->tcp_list->row(item));
                widget->deleteLater();
            }
            return true;
        }
    }
    return QWidget::eventFilter(obj, event);
}

void TCP::on_add_new_button_clicked()
{
    auto widget = addNewToolFrame("");
    widget->startEdit();
}

ListItemWidget* TCP::addNewToolFrame(const QString &name, const KDL::Frame& frame)
{
    QListWidgetItem* item = new QListWidgetItem(ui_->tcp_list);
    ListItemWidget* widget = new ListItemWidget(name, ui_->tcp_list->count(), ui_->tcp_list);
    widget->line_edit()->setReadOnly(true);
    item->setSizeHint(widget->sizeHint());
    ui_->tcp_list->addItem(item);
    ui_->tcp_list->setItemWidget(item, widget);
    connect(widget, &ListItemWidget::editFinished, this, [this, widget, frame](const auto& new_name, const auto& old_name){
        if(old_name.isEmpty()) {
            QListWidgetItem* item = findItemByWidget(ui_->tcp_list, widget);
            if (new_name.isEmpty() || tool_info_.count(new_name.toStdString())) {
                QMessageBox::warning(this, "Warning",
                                     new_name.isEmpty() ? "Empty name is not allowed!" : "Name repetition!");
                int row = ui_->tcp_list->row(item);
                delete ui_->tcp_list->takeItem(row);
                widget->deleteLater();
            }
            else {
                RobotHandle::instance().addToolFrame(new_name.toStdString(), frame);
            }
        }
        else {
            if (new_name.isEmpty() || (new_name != old_name && tool_info_.count(new_name.toStdString()))) {
                QMessageBox::warning(this, "Warning",
                                     new_name.isEmpty() ? "Empty name is not allowed!" : "Name repetition!");
                widget->setText(old_name);
            }
            else {
                const auto& src_frame = tool_info_.at(old_name.toStdString());
                RobotHandle::instance().deleteToolFrame(old_name.toStdString());
                RobotHandle::instance().addToolFrame(new_name.toStdString(), src_frame);
            }
        }
    });
    connect(widget->getCheckBox(), &QCheckBox::clicked, this, [widget](bool checked){
        if(checked) {
            RobotHandle::instance().setCurrentToolFrame(widget->text().toStdString());
            KinematicsPlugin::instance().refreshCoordinateSystem();
        }
    });
    check_box_group_->addButton(widget->getCheckBox());
    if(name.toStdString() == current_tool_name_) {
        widget->getCheckBox()->setChecked(true);
    }
    return widget;
}

QListWidgetItem* TCP::findItemByWidget(QListWidget* list, QWidget* w) {
    for (int i = 0; i < list->count(); ++i) {
        QListWidgetItem* item = list->item(i);
        if (list->itemWidget(item) == w) {
            return item;
        }
    }
    return nullptr;
}

void TCP::on_calibrate_button_clicked()
{
    auto name_list = PointPool::instance().getAllPointsName();
    std::vector<std::string> selected;
    StringSelectionDialog dlg(name_list);
    if (dlg.exec() == QDialog::Accepted) {
        selected = dlg.selectedItems();
        MovePointInfos points;
        for(const auto& s : selected) {
            points[s] = PointPool::instance().getPoint((s));
        }
        auto result = KinematicsPlugin::instance().tcpCalibration(points);
        auto widget = addNewToolFrame("", result);
        widget->startEdit();
    }

}

