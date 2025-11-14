#include "plot_widget.h"
#include <magic_enum/magic_enum.hpp>
#include <QDateTime>
#include <QTimeZone>

PlotWidget::PlotWidget(const DataTypeEnum& data_type, const size_t window_size, const std::string& joint_name, QWidget *parent)
    :
    QWidget(parent),
    data_type_(data_type),
    joint_name_(joint_name),
    window_size_(window_size),
    series_(new QLineSeries()),
    chart_(new QChart()),
    chart_view_(new QChartView(chart_)),
    axis_x_(new QDateTimeAxis()),
    axis_y_(new QValueAxis()),
    timer_(new QTimer(this))
{

    chart_->addSeries(series_);
    chart_->legend()->hide();
    chart_->addAxis(axis_x_, Qt::AlignBottom);
    chart_->addAxis(axis_y_, Qt::AlignLeft);
    series_->attachAxis(axis_x_);
    series_->attachAxis(axis_y_);
    chart_->setTitle(QString::fromStdString(joint_name_));
    axis_x_->setFormat("ss.zz");
    axis_x_->setTitleText("sample time(seconds)");
    axis_y_->setTitleText("value");
    chart_view_->setRenderHint(QPainter::Antialiasing);

    auto *layout = new QVBoxLayout(this);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    layout->addWidget(chart_view_);
    setLayout(layout);

    connect(timer_, &QTimer::timeout, this, &PlotWidget::pushData);
    timer_->start(20); // 20 ms -> 50 Hz
    elapsed_.start();
}

PlotWidget::~PlotWidget()
{
}

void PlotWidget::freeze(bool freeze)
{
    if(freeze) timer_->stop();
    else timer_->start();
}

bool PlotWidget::isRunning() const
{
    return timer_->isActive();
}

void PlotWidget::pushData()
{
    auto& db = DataBase::instance();
    auto data = db.getData(data_type_, joint_name_);
    auto new_ps = data.getSnapShot(window_size_);
    if(new_ps.empty()) {
        return;
    }
    QDateTime t = QDateTime::fromMSecsSinceEpoch(0, QTimeZone::utc());
    axis_y_->setRange(data.getMin() - 1e-3, data.getMax() + 1e-3);
    axis_x_->setRange(t.addMSecs(new_ps.front().x()), t.addMSecs(new_ps.back().x()));
    series_->replace(new_ps);
}
