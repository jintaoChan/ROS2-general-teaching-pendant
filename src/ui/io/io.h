#pragma once

#include "app_ports.h"

#include <controller_manager_msgs/srv/list_hardware_interfaces.hpp>
#include <rclcpp/rclcpp.hpp>

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QScrollArea>
#include <QVector>

#include <atomic>
#include <mutex>
#include <optional>
#include <unordered_map>

enum class IOType { INPUT, OUTPUT };

class IOPointWidget : public QWidget {
    Q_OBJECT
public:
    explicit IOPointWidget(const QString &name, IOType type, QWidget *parent = nullptr);

    void updateStatus(const std::optional<bool> &status);
    void setMonitorable(bool monitorable);

signals:
    void clicked(bool target_state);

private:
    IOType type_;
    QLabel *led_;
    QLabel *name_label_;
    QPushButton *on_button_{nullptr};
    QPushButton *off_button_{nullptr};
    bool current_status_{false};
    bool has_status_{false};
    bool monitorable_{true};
};

class IOModuleCard : public QGroupBox {
    Q_OBJECT
public:
    explicit IOModuleCard(const QString &module_name, IOType type, QWidget *parent = nullptr);
    void addIO(const QString &io_name);
    
    void setIOStatus(const QString &io_name, const std::optional<bool> &status);
    void setIOMonitorable(const QString &io_name, bool monitorable);

signals:
    void ioToggled(const QString &io_name, bool target_state);

private:
    IOType type_;
    QGridLayout *grid_layout_;
    QMap<QString, IOPointWidget*> io_;
    size_t io_count_{0};
};

class IOPanel : public QWidget {
    Q_OBJECT
public:
    explicit IOPanel(const AppPorts& ports, QWidget *parent = nullptr);
    ~IOPanel() override;
    
    void initLayout(IOStatus state);

private:
    void flushPendingState();
    void loadOutputInterfaceNames();
    bool parseGroupAndInterfaceName(const std::string &full_name,
                                    std::string &group_name,
                                    std::string &interface_name) const;
    QScrollArea *scroll_area_;
    QWidget *container_;
    QMap<QString, IOModuleCard*> cards_;
    std::unordered_map<std::string, std::vector<std::string>> output_group_interface_names_;
    std::unordered_map<std::string, std::vector<std::string>> output_group_no_feedback_interfaces_;
    rclcpp::Node::SharedPtr io_query_node_;
    rclcpp::Client<controller_manager_msgs::srv::ListHardwareInterfaces>::SharedPtr list_hw_if_client_;
    std::optional<size_t> io_status_callback_id_;
    IOStatus pending_state_;
    std::mutex pending_state_mutex_;
    std::atomic<bool> update_scheduled_{false};
    IRobotStateProvider* state_port_{nullptr};
    IRobotCommandPort* command_port_{nullptr};
    IRobotEvents* event_port_{nullptr};
};
