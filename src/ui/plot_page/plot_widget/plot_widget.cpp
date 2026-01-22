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
    timer_(new QTimer(this))
{
    window_size_ = std::min(window_size_, DataBase::instance().getSize());
    plot_ = new JKQTPlotter();
    graph_ = new JKQTPXYLineGraph(plot_);
    plot_->addGraph(graph_);
    column_x_ = plot_->getDatastore()->addColumn("time");
    column_y_ = plot_->getDatastore()->addColumn("value");
    graph_->setXColumn(column_x_);
    graph_->setYColumn(column_y_);
    graph_->setDrawLine(true);
    graph_->setLineWidth(1.0);
    graph_->setSymbolType(JKQTPNoSymbol);
    plot_->getPlotter()->setUseAntiAliasingForGraphs(false);
    plot_->getPlotter()->setUseAntiAliasingForSystem(false);
    plot_->getPlotter()->setUseAntiAliasingForText(false);
    plot_->getYAxis()->setLabelDigits(2);
    plot_->getYAxis()->setTickLabelType(JKQTPCALabelType::JKQTPCALTdefault);
    plot_->deregisterMouseDragAction(Qt::LeftButton, Qt::NoModifier);
    plot_->registerMouseDragAction(Qt::LeftButton, Qt::NoModifier, JKQTPMouseDragActions::jkqtpmdaPanPlotOnMove);
    plot_->deregisterMouseMoveAction(Qt::NoModifier);
    plot_->registerMouseMoveAction(Qt::NoModifier, JKQTPMouseMoveActions::jkqtpmmaToolTipForClosestDataPoint);
    plot_->deregisterMouseWheelAction(Qt::NoModifier);
    plot_->registerMouseWheelAction(Qt::NoModifier, JKQTPMouseWheelActions::jkqtpmwaZoomByWheel);
    auto *layout = new QVBoxLayout(this);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    layout->addWidget(plot_);
    setLayout(layout);

    connect(timer_, &QTimer::timeout, this, &PlotWidget::updateData);
    timer_->start(100); // 50 ms -> 20 Hz
    elapsed_.start();
}

PlotWidget::~PlotWidget()
{
}

void PlotWidget::freeze(bool freeze)
{
    if(freeze) timer_->stop();
    else {
        timer_->start();
        force_auto_scroll = true;
    }
}

bool PlotWidget::isRunning() const
{
    return timer_->isActive();
}

void PlotWidget::clear()
{
    plot_->getDatastore()->deleteColumn(column_x_);
    plot_->getDatastore()->deleteColumn(column_y_);
    column_x_ = plot_->getDatastore()->addColumn("time");
    column_y_ = plot_->getDatastore()->addColumn("value");
    graph_->setXColumn(column_x_);
    graph_->setYColumn(column_y_);
}

void PlotWidget::updateData()
{
    auto& db = DataBase::instance();
    auto data = db.getData(data_type_, joint_name_);
    auto new_ps = data.getSnapShot(last_update_head_, std::numeric_limits<size_t>::max() >> 1);
    auto last_head = last_update_head_;
    last_update_head_ = db.getCurrentIndex();
    if(new_ps.empty()) {
        return;
    }
    plot_->setPlotUpdateEnabled(false);
    JKQTPDatastore* ds = plot_->getDatastore();
    for (const auto& pt : new_ps) {
        ds->appendToColumn(column_x_, last_head++);
        ds->appendToColumn(column_y_, pt);
    }
    size_t current_count = ds->getRows(column_x_);
    if (current_count == 0) return;

    double data_max_x = current_count - 1;
    double current_view_max_x = plot_->getXMax();
    double snap_margin = window_size_ * 0.02; 

    if (current_view_max_x >= data_max_x - snap_margin) {
        is_auto_scroll_ = true;
    } else {
        if (QApplication::mouseButtons() & Qt::LeftButton) {
            if (force_auto_scroll){
                force_auto_scroll = false;
                is_auto_scroll_ = true;
            } else {
                is_auto_scroll_ = false;
            }
        }
    }
    if (is_auto_scroll_) {
        plot_->setX(std::max(0.0, data_max_x - window_size_), data_max_x);
    }
    plot_->zoomToFit(false, true);
    if(this->isVisible()){
        plot_->setPlotUpdateEnabled(true);
        plot_->redrawPlot();
    }
}
