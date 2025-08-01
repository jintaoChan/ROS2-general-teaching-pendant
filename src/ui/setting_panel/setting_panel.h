#ifndef SETTING_PANEL_H
#define SETTING_PANEL_H

#include <QWidget>

enum class ControlCoordinateSystemType {
    TOOL = 0,
    END_EFFECTOR
};

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
    double getStep() const;
    double getVelocity() const;
    ControlCoordinateSystemType getCoordinateSystem();

signals:
    void on_add_to_point_pool_button_clicked();

private slots:
    void on_coordinate_selection_combobox_currentIndexChanged(int index);

private:
    Ui::SettingPanel *ui;
    ControlCoordinateSystemType m_ControlCoordinateSystemType;
};

#endif // SETTING_PANEL_H
