#include <QClipboard>
#include "plot_page.h"
#include <stdexcept>

PlotPage::PlotPage(size_t window_size, IRobotStateProvider* state_port, QWidget *parent)
    :
    QWidget{parent},
    window_size_{window_size},
    main_layout_(new QVBoxLayout(this)),
    control_layout_(new QHBoxLayout()),
    graph_layout_(new QGridLayout()),
    save_button_(new QPushButton(this)),
    freeze_button_(new QPushButton(this)),
    clear_button_(new QPushButton(this)),
    combobox_(new QComboBox(this)),
    state_port_{state_port}
{
    if (state_port_ == nullptr) {
        throw std::invalid_argument("PlotPage requires non-null state port");
    }

    auto type_list = magic_enum::enum_values<DataTypeEnum>();
    for(const auto& t: type_list) {
        auto t_name = std::string(magic_enum::enum_name(t));
        combobox_->addItem(QString::fromStdString(t_name));
    }
    save_button_->setText("Save data to clip board");
    freeze_button_->setText("Freeze");
    clear_button_->setText("Clear");
    control_layout_->addWidget(combobox_);
    control_layout_->addWidget(save_button_);
    control_layout_->addWidget(freeze_button_);
    control_layout_->addWidget(clear_button_);
    checkbox_layout_ = new QHBoxLayout();
    main_layout_->addLayout(control_layout_);
    main_layout_->addLayout(checkbox_layout_);
    main_layout_->addLayout(graph_layout_);
    connect(combobox_, &QComboBox::currentTextChanged, this, &PlotPage::switchGraphContent);
    connect(save_button_, &QPushButton::pressed, this, &PlotPage::saveData);
    connect(freeze_button_, &QPushButton::pressed, this, &PlotPage::freezeWindow);
    connect(clear_button_, &QPushButton::pressed, this, &PlotPage::clear);
    switchGraphContent(combobox_->currentText());
}

void PlotPage::clearGraphs()
{
    if (graph_layout_ == nullptr)
        return;
    QLayoutItem *item;
    for(auto& g: graph_list_){
        delete g;
    }
    graph_list_.clear();
    while ((item = graph_layout_->takeAt(0)) != nullptr) {

        if (QWidget *widget = item->widget()) {
            widget->setParent(nullptr);
            delete widget;
        }
        delete item;
    }
}

void PlotPage::switchGraphContent(const QString &type_name)
{
    clearGraphs();
    // Clear old checkboxes
    for (auto* cb : joint_checkboxes_) delete cb;
    joint_checkboxes_.clear();

    auto type = magic_enum::enum_cast<DataTypeEnum>(type_name.toStdString()).value();
    auto joint_names = state_port_->getJointsName();

    auto g = new PlotWidget(type, window_size_, joint_names, this);
    graph_list_.push_back(g);
    graph_layout_->addWidget(g, 0, 0);
    graph_layout_->setRowStretch(0, 1);
    graph_layout_->setColumnStretch(0, 1);

    for (const auto& name : joint_names) {
        auto* cb = new QCheckBox(QString::fromStdString(name), this);
        cb->setChecked(true);
        checkbox_layout_->addWidget(cb);
        joint_checkboxes_.push_back(cb);
        connect(cb, &QCheckBox::toggled, this, [g, name](bool checked) {
            g->setJointVisible(name, checked);
        });
    }
}

void PlotPage::saveData()
{
    QClipboard *clipboard = QApplication::clipboard();
    clipboard->setText(QString::fromStdString(DataBase::instance().toPlainText()));
}

void PlotPage::freezeWindow()
{
    bool is_freeze = false;
    for(auto& g: graph_list_) {
        if(g->isRunning()){
            g->freeze(true);
            is_freeze = true;
        }
        else{
            g->freeze(false);
        }
    }
    if(is_freeze) {
        freeze_button_->setText("Unfreeze");
    }
    else{
        freeze_button_->setText("Freeze");
    }

}

void PlotPage::clear()
{
    DataBase::instance().clear();
    for(auto& g: graph_list_) {
        g->clear();
    }
}
