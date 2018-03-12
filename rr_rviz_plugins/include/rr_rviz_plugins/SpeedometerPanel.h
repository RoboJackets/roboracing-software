#ifndef PROJECT_SPEEDOMETERPANEL_H
#define PROJECT_SPEEDOMETERPANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <rr_platform/chassis_state.h>
#include <rr_platform/steering.h>
#include <QLabel>
#include <QtGui>
#include <QPainter>
#include <math.h>
#include <QString>



namespace rr_rviz_plugins {


class SpeedometerPanel : public rviz::Panel {




Q_OBJECT
public:


    SpeedometerPanel(QWidget *parent = 0);

protected:

    int height = 200;
    int width = 200;
    int maxSpeed = 3;

    double wheelAngle = 0;	
    void paintEvent(QPaintEvent* event) override;
    void tickMarkGenerator(QPainter &painter, float speeds);

    ros::Subscriber chassis_subscriber;
    ros::Subscriber steering_subscriber;

 
    void chassis_callback(const rr_platform::chassis_stateConstPtr &msg, QLabel *speedlabel);

    void steering_callback(const rr_platform::steeringConstPtr &msg); 


};

}

#endif 
