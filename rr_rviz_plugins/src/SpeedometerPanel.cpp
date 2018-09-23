#include <rr_rviz_plugins/SpeedometerPanel.h>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.h>

namespace rr_rviz_plugins {

SpeedometerPanel::SpeedometerPanel(QWidget *parent)
  : rviz::Panel(parent) 
{
    ros::NodeHandle handle;


    QLabel *speedlabel = new QLabel("0 m/s");


    chassis_subscriber = handle.subscribe<rr_platform::chassis_state>("/chassis_state", 1, boost::bind(&SpeedometerPanel::chassis_callback, this, _1, speedlabel));
    steering_subscriber = handle.subscribe<rr_platform::steering>("/steering", 1, &SpeedometerPanel::steering_callback, this); 	


    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(speedlabel);
    setLayout(layout);
}

void SpeedometerPanel::paintEvent(QPaintEvent* event)
{
 QPainter painter(this);

 widgetWidth = painter.viewport().width()/2;

 painter.drawLine(width,height, widgetWidth/2 + 70, widgetWidth/2 + 70);
 painter.drawArc(70,70,widgetWidth,widgetWidth,0,5760/2);

 for (float i = 0; i <= 3; i += 0.5) {
    tickMarkGenerator(painter, i);
 }

 painter.drawArc(70,widgetWidth,widgetWidth,widgetWidth,0,5760);
 painter.drawLine(widgetWidth/2 + 70 + (widgetWidth/4)*cos(wheelAngle + 3.14159265/3), widgetWidth*1.5-(widgetWidth/4)*sin(wheelAngle + 3.14159265/3), widgetWidth/2+70+(widgetWidth/2)*cos(wheelAngle +   3.14159265/3), widgetWidth*1.5-(widgetWidth/2)*sin(wheelAngle + 3.14159265/3));
 painter.drawLine(widgetWidth/2+70+(widgetWidth/4)*cos(wheelAngle + 2 * 3.14159265/3), widgetWidth*1.5-(widgetWidth/4)*sin(wheelAngle + 2 * 3.14159265/3), widgetWidth/2+70+(widgetWidth/2)*cos(wheelAngle + 2 * 3.14159265/3), widgetWidth*1.5-(widgetWidth/2)*sin(wheelAngle + 2 * 3.14159265/3));
}

void SpeedometerPanel::tickMarkGenerator(QPainter &painter, float speeds) {
    float speed = 3.1415 - (speeds/3) * 3.1415;

    painter.drawLine(cos(speed) * (widgetWidth/2 + widgetWidth/40) + widgetWidth/2 + 70, widgetWidth/2 + 70 - sin(speed) * (widgetWidth/2 + widgetWidth/40),cos(speed) * (widgetWidth/2 - widgetWidth/40) + widgetWidth/2 + 70, widgetWidth/2 + 70 - sin(speed) * (widgetWidth/2 - widgetWidth/40));
    
    painter.drawText(cos(speed) * (widgetWidth/2 + widgetWidth/8) - widgetWidth/40 + widgetWidth/2 + 70, widgetWidth/2 + 70 - sin(speed) * (widgetWidth/2 + widgetWidth/8), QString::number(speeds,'g',2));

}

void SpeedometerPanel::chassis_callback(const rr_platform::chassis_stateConstPtr &msg, QLabel *speedlabel) {
    
    auto text = std::to_string(msg->speed_mps) + " m/s (actual)";
    
    speedlabel->setText(text.c_str());
    
    double angle = ((msg->speed_mps)/maxSpeed)*3.14159256;

    width = widgetWidth/2 + 70 - (widgetWidth/2)* cos(angle);
    height = widgetWidth/2 + 70 - (widgetWidth/2)* sin(angle);
   	
    update();
}


void SpeedometerPanel::steering_callback(const rr_platform::steeringConstPtr &msg) {
   wheelAngle = msg->angle;
   update();
}

}


PLUGINLIB_EXPORT_CLASS( rr_rviz_plugins::SpeedometerPanel, rviz::Panel)
