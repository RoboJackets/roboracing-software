#include <rr_rviz_plugins/StartDetectPanel.h>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.h>

/*
 * Just as in the header, everything needs to happen in the rr_rviz_plugins namespace.
 */
namespace rr_rviz_plugins {

StartDetectPanel::StartDetectPanel(QWidget *parent)
  : rviz::Panel(parent) // Base class constructor
{
    // Panels are allowed to interact with NodeHandles directly just like ROS nodes.
    ros::NodeHandle handle;

    // Initialize a label for displaying some data
    QLabel *label = new QLabel("Wait");

    /* Initialize our subscriber to listen to the /speed topic.
    * Note the use of boost::bind, which allows us to give the callback a pointer to our UI label.
    */
    speed_subscriber = handle.subscribe<rr_platform::speed>("/speed", 1, boost::bind(&StartDetectPanel::speed_callback, this, _1, label));

    // StartDetect_subscriber = handle.subscribe<std_msgs::Bool>();

    /* Use QT layouts to add widgets to the panel.
    * Here, we're using a VBox layout, which allows us to stack all of our widgets vertically.
    */
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
    setLayout(layout);
}

void StartDetectPanel::speed_callback(const rr_platform::speedConstPtr &msg, QLabel *label) {
    // Create the new contents of the label based on the speed message.
    //auto text = std::to_string(msg->speed) + " m/s";

    if ((msg->speed) > 0) {
        label->setText("GO!");
    } else {
        label->setText("Wait");
    }
    // Set the contents of the label.
}

//void StartDetectPanel::paintEvent(QPaintEvent *event)  {
//
//}

}

/*
 * IMPORTANT! This macro must be filled out correctly for your panel class.
 */
PLUGINLIB_EXPORT_CLASS( rr_rviz_plugins::StartDetectPanel, rviz::Panel)
