#ifndef PLOT_WIDGET_H
#define PLOT_WIDGET_H

#include <QWidget>
#include <QtWidgets/QApplication>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QDateTimeAxis>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <QElapsedTimer>

#include "database.h"

class PlotWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PlotWidget(const DataTypeEnum& data_type, const size_t window_size, const std::string& joint_name, QWidget *parent = nullptr);
    ~PlotWidget();

public:
    void freeze(bool freeze);
    bool isRunning() const;

private slots:
    void pushData();

private:
    DataTypeEnum data_type_;
    std::string joint_name_;
    size_t window_size_;
    QLineSeries* series_;
    QChart* chart_;
    QChartView* chart_view_;
    QDateTimeAxis* axis_x_;
    QValueAxis* axis_y_;
    QTimer* timer_;

    QElapsedTimer elapsed_;
};

#endif // PLOT_WIDGET_H
