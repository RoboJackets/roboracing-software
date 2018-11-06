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

    // Initialize a label for displaying, shows StartDetect has yet to start publishing
    QLabel *label = new QLabel("Never Been Detected, (Not Publishing)");

    /* Initialize our subscriber to listen to the /start_detect topic.
    * Note the use of boost::bind, which allows us to give the callback a pointer to our UI label.
    */

    startDetect_subscriber = handle.subscribe<std_msgs::Bool>("/start_detected", 1, boost::bind(&StartDetectPanel::start_callback, this, _1, label));

    /* Use QT layouts to add widgets to the panel.
    * Here, we're using a VBox layout, which allows us to stack all of our widgets vertically.
    */
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
    setLayout(layout);
}

void StartDetectPanel::start_callback(const std_msgs::BoolConstPtr &msg, QLabel *label) {
    // Create the new contents of the label based on the  message.
    //auto tex

    if(msg->data) {
        label->setText("Publishing & Detected");
    } else {
        label->setText("Publishing, Not Detected");
    }
    // Set the contents of the label.
}


}

/*
 * IMPORTANT! This macro must be filled out correctly for your panel class.
 */
PLUGINLIB_EXPORT_CLASS( rr_rviz_plugins::StartDetectPanel, rviz::Panel)
