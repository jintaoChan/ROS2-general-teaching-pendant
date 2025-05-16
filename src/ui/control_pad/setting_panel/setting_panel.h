#ifndef SETTING_PANEL_H
#define SETTING_PANEL_H

#include <QWidget>

namespace Ui {
class SettingPanel;
}

class SettingPanel : public QWidget
{
    Q_OBJECT

public:
    explicit SettingPanel(QWidget *parent = nullptr);
    ~SettingPanel();

public:
    double getStep();
    double getVelocity();

signals:
    void on_add_to_point_pool_button_clicked();

private:
    Ui::SettingPanel *ui;
};

#endif // SETTING_PANEL_H
