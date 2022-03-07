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
    // Connect the update speed and start node methods
    QObject::connect(worker, &Worker::updateSpeed, this, &Speedometer::setLabel);
    QObject::connect(this, &Speedometer::startNode, worker, &Worker::startNode);

    // Make worker execute in separate thread
    worker->moveToThread(thread);
    connect(thread, &QThread::finished, thread, &QThread::deleteLater);

    // Initialize thread
    thread->start();

    // Tell the worker to start execution
    emit startNode();
}

/**
 * @brief Update label to include speed
 * @param speed the speed to display
 */
void Speedometer::setLabel(float speed) {
    auto text = std::to_string(speed) + " m/s";
    // Set the contents of the label.
    label->setText(text.c_str());
}

}  // namespace rr_rviz_plugins

PLUGINLIB_EXPORT_CLASS(rr_rviz_plugins::Speedometer, rviz_common::Panel)
