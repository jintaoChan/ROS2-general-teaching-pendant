#ifndef PLOT_PAGE_H
#define PLOT_PAGE_H

#include <QWidget>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QPushButton>
#include "database.h"
#include "plot_widget.h"

class PlotPage : public QWidget
{
    Q_OBJECT
public:
    explicit PlotPage(size_t window_size, QWidget *parent = nullptr);

signals:

private:
    void clearGraphs();


private slots:
    void switchGraphContent(const QString& type_name);
    void saveData();
    void freezeWindow();
private:
    size_t window_size_;
    QVBoxLayout* main_layout_;
    QHBoxLayout* control_layout_;
    QGridLayout* graph_layout_;
    QPushButton* save_button_;
    QPushButton* freeze_button_;
    QComboBox* combobox_;
    QList<PlotWidget*> graph_list_;
};

#endif // PLOT_PAGE_H
