#pragma once

#include <QTreeView>
#include <QStandardItem>
#include <QKeyEvent>
#include "robot_description.h"
#include "utils.h"



class TreeViewWithKeyEvent : public QTreeView
{
    Q_OBJECT

public:
    explicit TreeViewWithKeyEvent(QWidget *parent = nullptr);
    ~TreeViewWithKeyEvent(){};

protected:
    virtual void keyPressEvent(QKeyEvent *event) override;
public:
    virtual void deleteEvent(QModelIndex index) = 0;


private:
};


