#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>

namespace avc {
class start_detector : public nodelet::Nodelet {
private:
    ros::Publisher start_pub;
    image_transport::Subscriber img_sub;
    cv::Rect mask;

    void ImageCB(const sensor_msgs::ImageConstPtr &msg);

    virtual void onInit();
};

}


