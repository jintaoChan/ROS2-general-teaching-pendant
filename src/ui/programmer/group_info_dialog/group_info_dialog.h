#ifndef GROUP_INFO_DIALOG_H
#define GROUP_INFO_DIALOG_H

#include <QDialog>
#include <QLineEdit>
#include <QSpinBox>


class GroupInfoDialog : public QDialog {
    Q_OBJECT

public:
    explicit GroupInfoDialog(QWidget *parent = nullptr);

    QString getString() const;
    int getInteger() const;


private slots:
    void onOkClicked();

private:
    QLineEdit *m_StringEdit;
    QSpinBox *m_IntSpinBox;
};

#endif // GROUP_INFO_DIALOG_H
