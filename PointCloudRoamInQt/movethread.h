#pragma once

#ifndef MOVETHREAD_H
#define MOVETHREAD_H
#include <QObject>

class MoveThread:public QObject{
    Q_OBJECT

signals:
    void moveThread(QThread*);
};

#endif // MOVETHREAD_H
