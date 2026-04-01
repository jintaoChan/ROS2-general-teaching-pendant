#include "plot_widget.h"
#include <magic_enum/magic_enum.hpp>
#include <cmath>
#include <QDateTime>
#include <QTimeZone>

static const QColor kLineColors[] = {
    QColor(31, 119, 180),   // blue
    QColor(255, 127, 14),   // orange
    QColor(44, 160, 44),    // green
    QColor(214, 39, 40),    // red
    QColor(148, 103, 189),  // purple
    QColor(140, 86, 75),    // brown
    QColor(227, 119, 194),  // pink
    QColor(127, 127, 127),  // gray
    QColor(188, 189, 34),   // olive
    QColor(23, 190, 207),   // cyan
};
static constexpr size_t kNumColors = sizeof(kLineColors) / sizeof(kLineColors[0]);

PlotWidget::PlotWidget(const DataTypeEnum& data_type, const size_t window_size, const std::vector<std::string>& joint_names, QWidget *parent)
    :
    QWidget(parent),
    data_type_(data_type),
    window_size_(window_size),
    timer_(new QTimer(this))
{
    window_size_ = std::min(window_size_, DataBase::instance().getSize());
    plot_ = new JKQTPlotter();

    plot_->getPlotter()->setUseAntiAliasingForGraphs(false);
    plot_->getPlotter()->setUseAntiAliasingForSystem(false);
    plot_->getPlotter()->setUseAntiAliasingForText(false);
    plot_->getPlotter()->setShowKey(true);
    plot_->getYAxis()->setLabelDigits(2);
    plot_->getYAxis()->setTickLabelType(JKQTPCALabelType::JKQTPCALTdefault);
    plot_->deregisterMouseDragAction(Qt::LeftButton, Qt::NoModifier);
    plot_->registerMouseDragAction(Qt::LeftButton, Qt::NoModifier, JKQTPMouseDragActions::jkqtpmdaPanPlotOnMove);
    plot_->deregisterMouseMoveAction(Qt::NoModifier);
    plot_->registerMouseMoveAction(Qt::NoModifier, JKQTPMouseMoveActions::jkqtpmmaToolTipForClosestDataPoint);
    plot_->deregisterMouseWheelAction(Qt::NoModifier);
    plot_->registerMouseWheelAction(Qt::NoModifier, JKQTPMouseWheelActions::jkqtpmwaZoomByWheel);

    JKQTPDatastore* ds = plot_->getDatastore();
    for (size_t i = 0; i < joint_names.size(); ++i) {
        GraphInfo gi;
        gi.joint_name = joint_names[i];
        gi.column_x = ds->addColumn("time");
        gi.column_y = ds->addColumn(QString::fromStdString(joint_names[i]));
        gi.graph = new JKQTPXYLineGraph(plot_);
        gi.graph->setXColumn(gi.column_x);
        gi.graph->setYColumn(gi.column_y);
        gi.graph->setDrawLine(true);
        gi.graph->setLineWidth(1.5);
        gi.graph->setSymbolType(JKQTPNoSymbol);
        gi.graph->setTitle(QString::fromStdString(joint_names[i]));
        gi.graph->setColor(kLineColors[i % kNumColors]);
        plot_->addGraph(gi.graph);
        graphs_.push_back(gi);
    }

    auto *layout = new QVBoxLayout(this);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    layout->addWidget(plot_);
    setLayout(layout);

    connect(timer_, &QTimer::timeout, this, &PlotWidget::updateData);
    timer_->start(100);
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

void PlotWidget::clear()
{
    JKQTPDatastore* ds = plot_->getDatastore();
    for (auto& gi : graphs_) {
        ds->setColumnData(gi.column_x, QVector<double>());
        ds->setColumnData(gi.column_y, QVector<double>());
    }
}

void PlotWidget::setJointVisible(const std::string& joint_name, bool visible)
{
    for (auto& gi : graphs_) {
        if (gi.joint_name == joint_name) {
            gi.visible = visible;
            gi.graph->setVisible(visible);
            if (!visible) {
                JKQTPDatastore* ds = plot_->getDatastore();
                ds->setColumnData(gi.column_x, QVector<double>());
                ds->setColumnData(gi.column_y, QVector<double>());
            }
            break;
        }
    }
}

void PlotWidget::decimateRegion(const std::vector<double>& snapshot, size_t begin, size_t end,
                                size_t stride, size_t x_offset,
                                QVector<double>& out_x, QVector<double>& out_y)
{
    for (size_t i = begin; i < end; i += stride) {
        size_t bucket_end = std::min(i + stride, end);
        double best_val = snapshot[i];
        double best_abs = std::abs(best_val);
        for (size_t j = i + 1; j < bucket_end; ++j) {
            double a = std::abs(snapshot[j]);
            if (a > best_abs) {
                best_abs = a;
                best_val = snapshot[j];
            }
        }
        out_x.append(static_cast<double>(x_offset + i + (bucket_end - 1 - i) / 2));
        out_y.append(best_val);
    }
}

void PlotWidget::updateData()
{
    if (graphs_.empty()) return;

    auto& db = DataBase::instance();
    size_t current_size = db.getData(data_type_, graphs_[0].joint_name).getCurrentSize();
    if (current_size == 0) return;

    size_t render_begin, render_end;

    if (is_auto_scroll_) {
        render_end = current_size;
        render_begin = current_size > window_size_ ? current_size - window_size_ : 0;
    } else {
        // Only render the visible viewport + margin
        double view_min = plot_->getXMin();
        double view_max = plot_->getXMax();
        double view_width = view_max - view_min;
        double margin = view_width * 0.5;

        render_begin = (view_min > margin) ? static_cast<size_t>(view_min - margin) : 0;
        render_end = std::min(current_size, static_cast<size_t>(std::ceil(view_max + margin + 1)));
    }

    size_t decimate_count = render_end - render_begin;
    // getSnapShot(n) returns last n items, so fetch up to render_end from tail
    size_t fetch_from_tail = current_size - render_begin;
    size_t stride = std::max<size_t>(1, decimate_count / kRecentBudget);

    plot_->setPlotUpdateEnabled(false);
    JKQTPDatastore* ds = plot_->getDatastore();

    for (auto& gi : graphs_) {
        if (!gi.visible) continue;
        const auto& data = db.getData(data_type_, gi.joint_name);
        auto snapshot = data.getSnapShot(fetch_from_tail);
        if (snapshot.empty()) continue;

        QVector<double> display_x, display_y;
        size_t est_points = decimate_count / stride + 2;
        display_x.reserve(est_points);
        display_y.reserve(est_points);

        // snapshot[0] = render_begin, only decimate [0, decimate_count)
        decimateRegion(snapshot, 0, decimate_count, stride, render_begin, display_x, display_y);

        ds->setColumnData(gi.column_x, display_x);
        ds->setColumnData(gi.column_y, display_y);
    }

    double data_max_x = static_cast<double>(current_size - 1);
    double current_view_max_x = plot_->getXMax();
    double snap_margin = window_size_ * 0.02;

    if (current_view_max_x >= data_max_x - snap_margin) {
        is_auto_scroll_ = true;
    } else {
        if (QApplication::mouseButtons() & Qt::LeftButton) {
            is_auto_scroll_ = false;
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
