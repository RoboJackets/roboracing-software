#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>

namespace avc {
class finish_detector : public nodelet::Nodelet {
private:
    ros::Publisher crosses_pub;
    image_transport::Subscriber img_saver_sub;

    void ImageCB(const sensor_msgs::ImageConstPtr &msg);
    float red_low_h;
    float red_low_s;
    float red_low_v;

    float red_high_h;
    float red_high_s;
    float red_high_v;
    virtual void onInit();
};

}


