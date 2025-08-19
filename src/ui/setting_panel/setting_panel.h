#ifndef SETTING_PANEL_H
#define SETTING_PANEL_H

#include <QDoubleValidator>
#include <QWidget>
#include "robot_handle.h"

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

class EmptyOkDoubleValidator : public QDoubleValidator {
public:
    using QDoubleValidator::QDoubleValidator;

    QValidator::State validate(QString &input, int &pos) const override {
        if (input.isEmpty()) {
            return QValidator::Acceptable;
        }
        return QDoubleValidator::validate(input, pos);
    }
};



#endif // SETTING_PANEL_H
