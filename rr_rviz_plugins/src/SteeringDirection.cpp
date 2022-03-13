#include "SteeringDirection.hpp"

#include <cmath>

#include "qthread.h"

namespace rr_rviz_plugins {

SteeringDirection::SteeringDirection(QWidget *parent)
      : rviz_common::Panel(parent)  // Base class constructor
{
    // Initialize a label for displaying some data
    label = new QLabel("0 m/s");
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
    setLayout(layout);
    label->setText("10 m/s");

    QThread *thread = new QThread();
    worker = new SteeringWorker();
    // Connect the update angle and start node methods
    QObject::connect(worker, &SteeringWorker::updateAngle, this, &SteeringDirection::setLabel);
    QObject::connect(this, &SteeringDirection::startNode, worker, &SteeringWorker::startNode);

    // Make worker execute in separate thread
    worker->moveToThread(thread);
    connect(thread, &QThread::finished, thread, &QThread::deleteLater);

    // Initialize thread
    thread->start();

    // Tell the worker to start execution
    emit startNode();
}

/**
 * @brief Update label to include angle
 * @param angle the angle to display
 */
void SteeringDirection::setLabel(float angle) {
    auto text = std::to_string(angle) + " radians";
    // Set the contents of the label.
    label->setText(text.c_str());
}

}  // namespace rr_rviz_plugins

PLUGINLIB_EXPORT_CLASS(rr_rviz_plugins::SteeringDirection, rviz_common::Panel)
