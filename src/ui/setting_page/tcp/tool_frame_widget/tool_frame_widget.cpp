#include <QHBoxLayout>
#include <QVBoxLayout>
#include "tool_frame_widget.h"

ToolFrameWidget::ToolFrameWidget(QWidget *parent)
    :
    QWidget{parent}
{
    QVBoxLayout* v_layout = new QVBoxLayout(this);

    auto add_dof = [&](const QString& label_name) {
        QHBoxLayout* layout = new QHBoxLayout(this);
        QLabel* label = new QLabel(label_name, this);
        DoubleClickLineEdit* edit = new DoubleClickLineEdit(this);
        layout->addWidget(label);
        layout->addWidget(edit);
        value_map_[label_name.toStdString()] = edit;
        v_layout->addLayout(layout);

        edit->setReadOnly(true);
        edit->setFrame(false);
        edit->setAlignment(Qt::Alignment::enum_type::AlignHCenter);
        connect(edit, &DoubleClickLineEdit::editingFinished, this, [this] {
            emit(toolInfoModified(value_map_["x"]->text().toDouble(),
                                  value_map_["y"]->text().toDouble(),
                                  value_map_["z"]->text().toDouble(),
                                  value_map_["rx"]->text().toDouble(),
                                  value_map_["ry"]->text().toDouble(),
                                  value_map_["rz"]->text().toDouble()));
        });
    };

    add_dof("x");
    add_dof("y");
    add_dof("z");
    add_dof("rx");
    add_dof("ry");
    add_dof("rz");

    setLayout(v_layout);
}

void ToolFrameWidget::setValue(const double &x, const double &y, const double &z, const double &rx, const double &ry, const double &rz)
{
    value_map_["x"]->setText(QString::number(x));
    value_map_["y"]->setText(QString::number(y));
    value_map_["z"]->setText(QString::number(z));
    value_map_["rx"]->setText(QString::number(rx));
    value_map_["ry"]->setText(QString::number(ry));
    value_map_["rz"]->setText(QString::number(rz));
}

