#include <rr_rviz_plugins/SpeedometerPanel.h>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.h>


/*
 * Just as in the header, everything needs to happen in the rr_rviz_plugins namespace.
 */
namespace rr_rviz_plugins {



SpeedometerPanel::SpeedometerPanel(QWidget *parent)
  : rviz::Panel(parent) // Base class constructor
{

    // Panels are allowed to interact with NodeHandles directly just like ROS nodes.
    ros::NodeHandle handle;

    // Initialize a label for displaying some data
    QLabel *speedlabel = new QLabel("0 m/s");


    /* Initialize our subscriber to listen to the /speed topic.
    * Note the use of boost::bind, which allows us to give the callback a pointer to our UI label.
    */
    chassis_subscriber = handle.subscribe<rr_platform::chassis_state>("/chassis_state", 1, boost::bind(&SpeedometerPanel::chassis_callback, this, _1, speedlabel));
    steering_subscriber = handle.subscribe<rr_platform::steering>("/steering", 1, &SpeedometerPanel::steering_callback, this); 	

    /* Use QT layouts to add widgets to the panel.
    * Here, we're using a VBox layout, which allows us to stack all of our widgets vertically.
    */
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

 //this is for the ticks (every .5 m/s)


 // painter.drawLine(0+60,210+60,20+60,210+60);
 // QString *string_one = new QString("0");
 // painter.drawText(40,270,*string_one);

 


 // painter.drawLine(210-(190*.866)+60,210-190/2+60, 210-(210*.866)+60,210-105+60);
 // QString *string_two = new QString("0.5");
 // painter.drawText(210-(190*.866)+60-45,210-190/2+60-15,*string_two);

 

 for (float i = 0; i <= 3; i += 0.5) {
    tickMarkGenerator(painter, i);
 }


 // painter.drawLine(210-(190*.5)+60,210-190*.866+60, 210-(210*.5)+60,210-210*.866+60);
 // QString *string_three = new QString("1.0");
 // painter.drawText(210-(190*.4)+60-45,210-190*.5-40,*string_three);



 // painter.drawLine(210+60,210-190+60,210+60,210-210+60);
 // QString *string_four = new QString("1.5");
 // painter.drawText(260,47,*string_four);



 // painter.drawLine(210+(190*.5)+60,210-190*.866+60, 210+(210*.5)+60,210-210*.866+60);
 // QString *string_five = new QString("2.0");
 // painter.drawText(210+(190*.4)+60+25,210-190*.5-40,*string_five);




 // painter.drawLine(210+(190*.866)+60,210-190/2+60, 210+(210*.866)+60,210-105+60);
 // QString *string_six = new QString("2.5");
 // painter.drawText(210+(190*.866)+45+45,210-190/2+60-15,*string_six);


 // painter.drawLine(210+190+60,210+60,420+60,210+60);
 // QString *string_seven = new QString("3.0");
 // painter.drawText(493,270,*string_seven);


 painter.drawArc(70,400,400,400,0,5760);
 painter.drawLine(270+100*cos(wheelAngle + 3.14159265/3), 600-100*sin(wheelAngle + 3.14159265/3), 270+200*cos(wheelAngle + 3.14159265/3), 600-200*sin(wheelAngle + 3.14159265/3));
 painter.drawLine(270+100*cos(wheelAngle + 2 * 3.14159265/3), 600-100*sin(wheelAngle + 2 * 3.14159265/3), 270+200*cos(wheelAngle + 2 * 3.14159265/3), 600-200*sin(wheelAngle + 2 * 3.14159265/3));
 

}


//thing to make tick marks - finish this
void SpeedometerPanel::tickMarkGenerator(QPainter &painter, float speeds) {
    float speed = 3.1415 - (speeds/3) * 3.1415;
    
    painter.drawLine(cos(speed) * 210 + 270, 270 - sin(speed) * 210,cos(speed) * 190 + 270, 270 - sin(speed) * 190);
    
    painter.drawText(cos(speed) * 225 - 10 + 270, 270 - sin(speed) * 225, QString::number(speeds,'g',1));

}

void SpeedometerPanel::chassis_callback(const rr_platform::chassis_stateConstPtr &msg, QLabel *speedlabel) {
    // Create the new contents of the label based on the speed message.
    auto text = std::to_string(msg->speed_mps) + " m/s (actual)";
    // Set the contents of the label.
    speedlabel->setText(text.c_str());

    //calculations for height and width
    double angle = ((msg->speed_mps)/maxSpeed)*3.14159256;
    width = 60+210-(200*cos(angle));
    height = 60+210-200*sin(angle);
	
    //wheelAngle = 1;
    
    	
    update();
}


void SpeedometerPanel::steering_callback(const rr_platform::steeringConstPtr &msg) {
   wheelAngle = msg->angle;
   update();
}

//void ExamplePanel::paintEvent(QPaintEvent *event)  {
//
//}

}

/*
 * IMPORTANT! This macro must be filled out correctly for your panel class.
 */
PLUGINLIB_EXPORT_CLASS( rr_rviz_plugins::SpeedometerPanel, rviz::Panel)
