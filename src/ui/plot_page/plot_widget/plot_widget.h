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
    explicit PlotWidget(const DataTypeEnum& data_type, const size_t window_size, const std::vector<std::string>& joint_names, QWidget *parent = nullptr);
    ~PlotWidget();

public:
    void freeze(bool freeze);
    bool isRunning() const;
    void clear();
    void setJointVisible(const std::string& joint_name, bool visible);

private slots:
    void updateData();

private:
    struct GraphInfo {
        std::string joint_name;
        JKQTPXYLineGraph* graph;
        size_t column_x;
        size_t column_y;
        bool visible = true;
    };

    static constexpr size_t kRecentBudget = 1500;

    void decimateRegion(const std::vector<double>& snapshot, size_t begin, size_t end,
                        size_t stride, size_t x_offset,
                        QVector<double>& out_x, QVector<double>& out_y);

    DataTypeEnum data_type_;
    size_t window_size_;
    JKQTPlotter* plot_;
    std::vector<GraphInfo> graphs_;

    QTimer* timer_;

    QElapsedTimer elapsed_;
    bool is_auto_scroll_ = true;
};

#endif // PLOT_WIDGET_H
