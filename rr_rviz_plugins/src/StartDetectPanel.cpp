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
    // speed_subscriber = handle.subscribe<rr_platform::speed>("/speed", 1, boost::bind(&StartDetectPanel::speed_callback, this, _1, label));

    startDetect_subscriber = handle.subscribe<std_msgs::Bool>("/start_detected", 1, boost::bind(&StartDetectPanel::start_callback, this, _1, label));

    /* Use QT layouts to add widgets to the panel.
    * Here, we're using a VBox layout, which allows us to stack all of our widgets vertically.
    */
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
    setLayout(layout);
}

void StartDetectPanel::start_callback(const std_msgs::BoolConstPtr &msg, QLabel *label) {
    // Create the new contents of the label based on the speed message.
    //auto text = std::to_string(msg->speed) + " m/s";

    if(msg->data) {
        label->setText("GO!");
    } else {
        label->setText("Wait");
    }
    // Set the contents of the label.
}

void StartDetectPanel::paintEvent(QPaintEvent *event)  {
    QColor background;
    // QColor crosshair;
    if( isEnabled() ) {
      background = Qt::white;
      // crosshair = Qt::black;
    } else {
      background = Qt::lightGray;
      // crosshair = Qt::darkGray;
    }
}

}

/*
 * IMPORTANT! This macro must be filled out correctly for your panel class.
 */
PLUGINLIB_EXPORT_CLASS( rr_rviz_plugins::StartDetectPanel, rviz::Panel)
