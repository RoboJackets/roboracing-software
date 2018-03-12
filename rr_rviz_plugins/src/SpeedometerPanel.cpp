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
 painter.drawLine(width,height, 270,270);
 painter.drawArc(70,70,400,400,0,5760/2);
 painter.drawLine(0+60,270,480,270);


 for (float i = 0; i <= 3; i += 0.5) {
    tickMarkGenerator(painter, i);
 }





 painter.drawArc(70,400,400,400,0,5760);
 painter.drawLine(270+100*cos(wheelAngle + 3.14159265/3), 600-100*sin(wheelAngle + 3.14159265/3), 270+200*cos(wheelAngle + 3.14159265/3), 600-200*sin(wheelAngle + 3.14159265/3));
 painter.drawLine(270+100*cos(wheelAngle + 2 * 3.14159265/3), 600-100*sin(wheelAngle + 2 * 3.14159265/3), 270+200*cos(wheelAngle + 2 * 3.14159265/3), 600-200*sin(wheelAngle + 2 * 3.14159265/3));
 

}



void SpeedometerPanel::tickMarkGenerator(QPainter &painter, float speeds) {
    float speed = 3.1415 - (speeds/3) * 3.1415;
    
    painter.drawLine(cos(speed) * 210 + 270, 270 - sin(speed) * 210,cos(speed) * 190 + 270, 270 - sin(speed) * 190);
    
    painter.drawText(cos(speed) * 225 - 10 + 270, 270 - sin(speed) * 225, QString::number(speeds,'g',1));

}

void SpeedometerPanel::chassis_callback(const rr_platform::chassis_stateConstPtr &msg, QLabel *speedlabel) {
    
    auto text = std::to_string(msg->speed_mps) + " m/s (actual)";
    
    speedlabel->setText(text.c_str());

    
    double angle = ((msg->speed_mps)/maxSpeed)*3.14159256;
    width = 60+210-(200*cos(angle));
    height = 60+210-200*sin(angle);
	
   
    
    	
    update();
}


void SpeedometerPanel::steering_callback(const rr_platform::steeringConstPtr &msg) {
   wheelAngle = msg->angle;
   update();
}



}


PLUGINLIB_EXPORT_CLASS( rr_rviz_plugins::SpeedometerPanel, rviz::Panel)
