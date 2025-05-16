#include "group_info_dialog.h"
#include <QLineEdit>
#include <QSpinBox>
#include <QDialogButtonBox>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QFormLayout>

GroupInfoDialog::GroupInfoDialog(QWidget *parent) : QDialog(parent) {
    m_StringEdit = new QLineEdit(this);
    m_IntSpinBox = new QSpinBox(this);
    m_IntSpinBox->setRange(-1, 10000);

    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow("Please input group name:", m_StringEdit);
    formLayout->addRow("Please input recycle times of this group:(-1 means infinity)", m_IntSpinBox);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &GroupInfoDialog::onOkClicked);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);

    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    mainLayout->addLayout(formLayout);
    mainLayout->addWidget(buttonBox);
}

QString GroupInfoDialog::getString() const {
    return m_StringEdit->text();
}

int GroupInfoDialog::getInteger() const {
    return m_IntSpinBox->value();
}

void GroupInfoDialog::onOkClicked()
{
    if (m_StringEdit->text().trimmed().isEmpty()) {
        QMessageBox::warning(this, "Warning", "Empty name is not allowed!");
        reject();
    } else {
        accept();
    }
}
