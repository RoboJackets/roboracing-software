#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>

namespace avc {
class finish_detector : public nodelet::Nodelet {
private:
    ros::Publisher finish_pub;
    image_transport::Subscriber img_sub;

    void ImageCB(const sensor_msgs::ImageConstPtr &msg);

    virtual void onInit();
};

}


