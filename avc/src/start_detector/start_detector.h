#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>

namespace avc {
class start_detector : public nodelet::Nodelet {
private:
    ros::Publisher start_pub;
    image_transport::Subscriber img_sub;


    double red_low_r;
    double red_low_g;
    double red_low_b;
    
    double red_high_r;
    double red_high_g;
    double red_high_b;

    int dp;
    int minDist;
    int param1;
    int param2;
    int minSize;
    int maxSize;

    int sumThreshold;

    void ImageCB(const sensor_msgs::ImageConstPtr &msg);

    virtual void onInit();
};

}


