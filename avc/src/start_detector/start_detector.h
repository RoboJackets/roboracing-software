#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

namespace avc {
class start_detector : public nodelet::Nodelet {
private:
    ros::Publisher start_pub;
    ros::Publisher debug_pub;
    image_transport::Subscriber img_sub;

    cv::Scalar red_low;
    cv::Scalar red_high;

    int dp;
    int minDist;
    int param1;
    int param2;
    int minSize;
    int maxSize;

    int sumThreshold;

    std::vector<double> detection_ring_buffer;

    void ImageCB(const sensor_msgs::ImageConstPtr &msg);

    virtual void onInit();
};

}


