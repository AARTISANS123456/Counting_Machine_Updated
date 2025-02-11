#pragma once
#ifndef WORKER_H
#define WORKER_H

#include <QObject>
#include <QTimer>
#include <functional>

class Worker : public QObject {
    Q_OBJECT

public:
    explicit Worker(std::function<void()> task, int interval, QObject* parent = nullptr)
        : QObject(parent), task(task), interval(interval) {
        timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, &Worker::onTimeout);
        timer->start(interval);
    }

private slots:
    void onTimeout() {
        task();
    }

private:
    QTimer* timer;
    std::function<void()> task;
    int interval;
};

#endif // WORKER_H
