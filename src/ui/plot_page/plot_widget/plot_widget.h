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
#include "jkqtplotter/jkqtplotter.h"
#include "jkqtplotter/graphs/jkqtplines.h"

class PlotWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PlotWidget(const DataTypeEnum& data_type, const size_t window_size, const std::string& joint_name, QWidget *parent = nullptr);
    ~PlotWidget();

public:
    void freeze(bool freeze);
    bool isRunning() const;
    void clear();

private slots:
    void updateData();

private:
    DataTypeEnum data_type_;
    std::string joint_name_;
    size_t window_size_;
    JKQTPlotter* plot_;
    JKQTPXYLineGraph* graph_;
    size_t column_x_;
    size_t column_y_;

    QTimer* timer_;

    QElapsedTimer elapsed_;
    size_t last_update_head_{0};
    bool is_auto_scroll_ = true;
    bool force_auto_scroll = false;
};

#endif // PLOT_WIDGET_H
