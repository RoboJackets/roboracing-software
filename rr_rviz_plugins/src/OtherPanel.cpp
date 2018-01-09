#include <rr_rviz_plugins/OtherPanel.h>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.h>


/*
 * Just as in the header, everything needs to happen in the rr_rviz_plugins namespace.
 */
namespace rr_rviz_plugins {



OtherPanel::OtherPanel(QWidget *parent)
  : rviz::Panel(parent) // Base class constructor
{

    // Panels are allowed to interact with NodeHandles directly just like ROS nodes.
    ros::NodeHandle handle;

    // Initialize a label for displaying some data
    QLabel *speedlabel = new QLabel("0 m/s");


    /* Initialize our subscriber to listen to the /speed topic.
    * Note the use of boost::bind, which allows us to give the callback a pointer to our UI label.
    */
    chassis_subscriber = handle.subscribe<rr_platform::chassis_state>("/chassis_state", 1, boost::bind(&OtherPanel::chassis_callback, this, _1, speedlabel));

    /* Use QT layouts to add widgets to the panel.
    * Here, we're using a VBox layout, which allows us to stack all of our widgets vertically.
    */
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(speedlabel);
    setLayout(layout);
    

}

void OtherPanel::paintEvent(QPaintEvent*)
{
 QPainter painter(this);
 painter.drawLine(width,height, 210,210);
 painter.drawArc(10,10,400,400,0,5760/2);
 painter.drawLine(0,210,420,210);

 //this is for the lines every 10mph
 painter.drawLine(0,210,20,210);
 painter.drawLine(210-(190*.866),210-190/2, 210-(210*.866),210-105);
 painter.drawLine(210-(190*.5),210-190*.866, 210-(210*.5),210-210*.866);
 painter.drawLine(210,210-190,210,210-210);
 painter.drawLine(210+(190*.5),210-190*.866, 210+(210*.5),210-210*.866);
 painter.drawLine(210+(190*.866),210-190/2, 210+(210*.866),210-105);
 painter.drawLine(210+190,210,420,210);
 

}

void OtherPanel::chassis_callback(const rr_platform::chassis_stateConstPtr &msg, QLabel *speedlabel) {
    // Create the new contents of the label based on the speed message.
    auto text = std::to_string(msg->speed_mps) + " m/s (actual)";
    // Set the contents of the label.
    speedlabel->setText(text.c_str());

    //calculations for height and width
    double angle = ((msg->speed_mps)/maxSpeed)*3.14159256;
    width = 210-(200*cos(angle));
    height = 210-200*sin(angle);
    update();
}

//void ExamplePanel::paintEvent(QPaintEvent *event)  {
//
//}

}

/*
 * IMPORTANT! This macro must be filled out correctly for your panel class.
 */
PLUGINLIB_EXPORT_CLASS( rr_rviz_plugins::OtherPanel, rviz::Panel)
