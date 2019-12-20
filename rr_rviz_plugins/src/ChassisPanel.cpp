//
// Created by root on 12/20/19.
//
#include <pluginlib/class_list_macros.h>
#include <rr_rviz_plugins/ChassisPanel.h>
#include <QVBoxLayout>
#include <QtGui/QPaintEvent>
#include <QtGui/QPainter>
#include <QtWidgets/QLabel>

namespace rr_rviz_plugins {
    QLabel *label;
ChassisPanel::ChassisPanel(QWidget *parent)
        : rviz::Panel(parent)
{
    auto *layout = new QVBoxLayout;
    label = new QLabel("No message");
    chassis_sub = nh.subscribe("/chassis_state",1,ChassisPanel::chassisStateCallback);
    layout->addWidget(label);
    setLayout(layout);
}
void ChassisPanel::chassisStateCallback(auto msg) {

}
}  // namespace rr_rviz_plugins

PLUGINLIB_EXPORT_CLASS(rr_rviz_plugins::ChassisPanel, rviz::Panel)