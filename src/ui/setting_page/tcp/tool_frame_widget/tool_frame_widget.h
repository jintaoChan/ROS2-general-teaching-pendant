#ifndef TOOL_FRAME_WIDGET_H
#define TOOL_FRAME_WIDGET_H

#include <QWidget>
#include <QLabel>
#include <QLineEdit>

class DoubleClickLineEdit : public QLineEdit {
    Q_OBJECT
public:
    DoubleClickLineEdit(QWidget *parent = nullptr) : QLineEdit(parent) {}

protected:
    void mouseDoubleClickEvent(QMouseEvent* event) override {
        if(!text().isEmpty()) {
            setReadOnly(false);
            setFocus();
            selectAll();
            QWidget::mouseDoubleClickEvent(event);
        }
    }
};

class ToolFrameWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ToolFrameWidget(QWidget *parent = nullptr);
    void setValue(const double& x, const double& y, const double& z, const double& rx, const double& ry, const double& rz);

signals:
    void toolInfoModified(const double& x, const double& y, const double& z, const double& rx, const double& ry, const double& rz);

private:
    std::unordered_map<std::string, DoubleClickLineEdit*> value_map_;
};

#endif // TOOL_FRAME_WIDGET_H
