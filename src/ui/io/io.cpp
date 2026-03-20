#include "io.h"
#include <QStyle>
#include <QLayout>
#include <QDebug>

#include <chrono>

IOPointWidget::IOPointWidget(const QString &name, IOType type, QWidget *parent)
    : QWidget(parent), type_(type)
{
    QHBoxLayout *layout = new QHBoxLayout(this);
    led_ = new QLabel();
    led_->setFixedSize(16, 16);
    updateStatus(std::nullopt);

    name_label_ = new QLabel(name);
    layout->addWidget(led_);
    layout->addWidget(name_label_);

    if (type_ == IOType::OUTPUT) {
        on_button_ = new QPushButton("On");
        off_button_ = new QPushButton("Off");
        layout->addWidget(on_button_);
        layout->addWidget(off_button_);

        connect(on_button_, &QPushButton::clicked, this, [this](){
            this->current_status_ = true;
            emit clicked(true);
        });
        connect(off_button_, &QPushButton::clicked, this, [this](){
            this->current_status_ = false;
            emit clicked(false);
        });
    }
    layout->addStretch();
}

void IOPointWidget::updateStatus(const std::optional<bool> &status) {
    if (!this->monitorable_) {
        return;
    }

    if (!status.has_value()) {
        this->has_status_ = false;
        led_->setStyleSheet("background-color: #95a5a6; border-radius: 8px; border: 1px solid #7f8c8d;");
        return;
    }

    if (this->has_status_ && this->current_status_ == status.value()) {
        return;
    }

    this->current_status_ = status.value();
    this->has_status_ = true;
    const QString color = status.value() ? "#2ecc71" : "#e74c3c";
    led_->setStyleSheet(QString("background-color: %1; border-radius: 8px; border: 1px solid #7f8c8d;").arg(color));
}

void IOPointWidget::setMonitorable(bool monitorable)
{
    if (this->monitorable_ == monitorable) {
        return;
    }

    this->monitorable_ = monitorable;
    if (!this->monitorable_) {
        this->has_status_ = false;
        led_->setStyleSheet("background-color: #95a5a6; border-radius: 8px; border: 1px solid #7f8c8d;");
        return;
    }

    this->has_status_ = false;
    updateStatus(std::nullopt);
}

IOModuleCard::IOModuleCard(const QString &module_name, IOType type, QWidget *parent)
    : QGroupBox(module_name, parent), type_(type)
{
    grid_layout_ = new QGridLayout(this);
    this->setStyleSheet(type_ == IOType::INPUT ?
        "IOModuleCard { border: 2px solid #3498db; margin-top: 10px; font-weight: bold; }" :
        "IOModuleCard { border: 2px solid #1abc9c; margin-top: 10px; font-weight: bold; }");
}

void IOModuleCard::addIO(const QString &io_name) {
    IOPointWidget *w = new IOPointWidget(io_name, type_, this);
    grid_layout_->addWidget(w, io_count_ / 2, io_count_ % 2);
    ++io_count_;
    io_[io_name] = w;

    if (type_ == IOType::OUTPUT) {
        connect(w, &IOPointWidget::clicked, this, [this, io_name](bool target_state) {
            emit ioToggled(io_name, target_state);
        });
    }
}

void IOModuleCard::setIOStatus(const QString &io_name, const std::optional<bool> &status) {
    auto it = io_.find(io_name);
    if(it != io_.end()) {
        it.value()->updateStatus(status);
    }
}

void IOModuleCard::setIOMonitorable(const QString &io_name, bool monitorable)
{
    auto it = io_.find(io_name);
    if(it != io_.end()) {
        it.value()->setMonitorable(monitorable);
    }
}

IOPanel::IOPanel(QWidget *parent) : QWidget(parent) {
    QVBoxLayout* main_layout = new QVBoxLayout(this);

    auto createLegendItem = [](const QString &color, const QString &text) {
        QWidget *item = new QWidget();
        QHBoxLayout *item_layout = new QHBoxLayout(item);
        item_layout->setContentsMargins(0, 0, 0, 0);
        item_layout->setSpacing(6);

        QLabel *dot = new QLabel();
        dot->setFixedSize(12, 12);
        dot->setStyleSheet(QString("background-color: %1; border-radius: 6px; border: 1px solid #7f8c8d;").arg(color));

        QLabel *label = new QLabel(text);
        item_layout->addWidget(dot);
        item_layout->addWidget(label);

        return item;
    };

    QWidget *legend = new QWidget(this);
    QHBoxLayout *legend_layout = new QHBoxLayout(legend);
    legend_layout->setContentsMargins(0, 0, 0, 0);
    legend_layout->setSpacing(16);
    legend_layout->addWidget(createLegendItem("#2ecc71", "ON"));
    legend_layout->addWidget(createLegendItem("#e74c3c", "OFF"));
    legend_layout->addWidget(createLegendItem("#95a5a6", "UNKNOWN"));
    legend_layout->addStretch();
    main_layout->addWidget(legend);
    
    scroll_area_ = new QScrollArea(this);
    scroll_area_->setWidgetResizable(true);
    
    container_ = new QWidget();
    QGridLayout* flow_layout = new QGridLayout(container_);
    container_->setLayout(flow_layout);

    loadOutputInterfaceNamesFromRobotHandle();
    
    scroll_area_->setWidget(container_);
    main_layout->addWidget(scroll_area_);
    io_status_callback_id_ = RobotHandle::instance().registerIOStatusCallback([this](IOStatus state){
        {
            std::lock_guard<std::mutex> lock(pending_state_mutex_);
            pending_state_ = std::move(state);
        }

        if (update_scheduled_.exchange(true)) {
            return;
        }

        QMetaObject::invokeMethod(this, [this]() {
            flushPendingState();
        }, Qt::QueuedConnection);
    });

}

IOPanel::~IOPanel()
{
    if (io_status_callback_id_.has_value()) {
        RobotHandle::instance().unregisterIOStatusCallback(io_status_callback_id_.value());
        io_status_callback_id_ = std::nullopt;
    }
}

void IOPanel::loadOutputInterfaceNamesFromRobotHandle()
{
    const auto &output_groups = RobotHandle::instance().getIOOutputGroupsName();
    output_group_interface_names_.clear();
    output_group_no_feedback_interfaces_.clear();
    for (const auto &group : output_groups) {
        const auto &ordered_interfaces = RobotHandle::instance().getIOInterfacesName(group);
        auto &display_interfaces = output_group_interface_names_[group];
        auto &no_feedback_interfaces = output_group_no_feedback_interfaces_[group];
        for (const auto &interface_name : ordered_interfaces) {
            display_interfaces.push_back(interface_name);
            if (!RobotHandle::instance().isIOMonitorable(group, interface_name)) {
                no_feedback_interfaces.push_back(interface_name);
            }
        }
    }
}

bool IOPanel::parseGroupAndInterfaceName(const std::string &full_name,
                                         std::string &group_name,
                                         std::string &interface_name) const
{
    const auto separator_pos = full_name.find('/');
    if (separator_pos == std::string::npos || separator_pos == 0 || separator_pos + 1 >= full_name.size()) {
        return false;
    }

    group_name = full_name.substr(0, separator_pos);
    interface_name = full_name.substr(separator_pos + 1);
    return true;
}

void IOPanel::flushPendingState()
{
    while (true) {
        IOStatus state;
        {
            std::lock_guard<std::mutex> lock(pending_state_mutex_);
            state = pending_state_;
        }

        initLayout(state);
        for(const auto& s : state) {
            const auto& module_name = QString::fromStdString(s.first);
            auto card_it = cards_.find(module_name);
            if (card_it == cards_.end()) {
                continue;
            }

            IOModuleCard *c = card_it.value();
            for(const auto& inf: s.second) {
                const auto& inf_name = QString::fromStdString(inf.first);
                c->setIOStatus(inf_name, inf.second);
            }
        }

        std::lock_guard<std::mutex> lock(pending_state_mutex_);
        if (state == pending_state_) {
            update_scheduled_ = false;
            return;
        }
    }
}

void IOPanel::initLayout(IOStatus state) {
    static std::once_flag flag;
    std::call_once(flag, [&](){
        const auto& input_names = RobotHandle::instance().getIOInputGroupsName();
        const auto& output_names = RobotHandle::instance().getIOOutputGroupsName();
        IOType t;
        size_t card_num = 0;
        for(const auto& m: state) {
            if(std::find(input_names.begin(), input_names.end(), m.first) != input_names.end()) {
                t = IOType::INPUT;
            }
            else {
                t = IOType::OUTPUT;
            }
            IOModuleCard *card = new IOModuleCard(QString::fromStdString(m.first), t, container_);
            if (t == IOType::OUTPUT) {
                const QString module_name = QString::fromStdString(m.first);
                connect(card, &IOModuleCard::ioToggled, this,
                        [module_name](const QString &io_name, bool target_state) {
                            RobotHandle::instance().setIOState(module_name.toStdString(),
                                                               io_name.toStdString(),
                                                               target_state);
                        });
            }
            const bool is_output_module =
                std::find(output_names.begin(), output_names.end(), m.first) != output_names.end();
            auto output_it = output_group_interface_names_.find(m.first);
            if (is_output_module && output_it != output_group_interface_names_.end() && !output_it->second.empty()) {
                std::vector<std::string> no_feedback_interfaces;
                auto no_feedback_it = output_group_no_feedback_interfaces_.find(m.first);
                if (no_feedback_it != output_group_no_feedback_interfaces_.end()) {
                    no_feedback_interfaces = no_feedback_it->second;
                }

                std::vector<std::string> added_interfaces;
                auto add_output_io = [&](const std::string &interface_name) {
                    if (std::find(added_interfaces.begin(), added_interfaces.end(), interface_name) != added_interfaces.end()) {
                        return;
                    }

                    const QString io_name = QString::fromStdString(interface_name);
                    card->addIO(io_name);
                    if (std::find(no_feedback_interfaces.begin(), no_feedback_interfaces.end(), interface_name) !=
                        no_feedback_interfaces.end()) {
                        card->setIOMonitorable(io_name, false);
                    }
                    added_interfaces.push_back(interface_name);
                };

                // Priority 1: follow RobotHandle callback message order (state order).
                for (const auto &inf : m.second) {
                    if (std::find(output_it->second.begin(), output_it->second.end(), inf.first) != output_it->second.end()) {
                        add_output_io(inf.first);
                    }
                }

                // Priority 2: append command-only interfaces not present in state message.
                for (const auto &interface_name : output_it->second) {
                    add_output_io(interface_name);
                }
            } else {
                for(const auto& inf: m.second) {
                    card->addIO(QString::fromStdString(inf.first));
                }
            }
            static_cast<QGridLayout*>(container_->layout())->addWidget(card, card_num / 2, card_num % 2);
            cards_[QString::fromStdString(m.first)] = card;
            ++card_num;
        }
    });
}
