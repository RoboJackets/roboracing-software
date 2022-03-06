#include "Speedometer.hpp"
#include "qthread.h"

#include <cmath>

namespace rr_rviz_plugins {

Speedometer::Speedometer(QWidget *parent)
      : rviz_common::Panel(parent)  // Base class constructor
{
    // Initialize a label for displaying some data
    label = new QLabel("0 m/s");
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
    setLayout(layout);
    label->setText("10 m/s");

    QThread *thread = new QThread();
    worker = new Worker();
    QObject::connect(worker, &Worker::updateSpeed, this, &Speedometer::setLabel);
    QObject::connect(this, &Speedometer::startNode, worker, &Worker::startNode);
    worker->moveToThread(thread);
    // connect( worker, &Worker::error, this, &MyClass::errorString);
    // connect( thread, &QThread::started, worker, &Worker::process);
    // connect( worker, &Worker::finished, thread, &QThread::quit);
    // connect( worker, &Worker::finished, worker, &Worker::deleteLater);
    connect( thread, &QThread::finished, thread, &QThread::deleteLater);
    thread->start();
    emit startNode();
}

void Speedometer::setLabel(float speed) {
    auto text = std::to_string(speed) + " m/s";
    // Set the contents of the label.
    label->setText(text.c_str());
}

}  // namespace rr_rviz_plugins

PLUGINLIB_EXPORT_CLASS(rr_rviz_plugins::Speedometer, rviz_common::Panel)
