#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace iarrc {
class color_detector : public nodelet::Nodelet {
private:
    image_transport::Publisher img_pub;
    image_transport::Subscriber img_sub;
    cv::Rect mask;

    cv::Mat erosion_kernel_blue;
    cv::Mat erosion_kernel_white;
    cv::Mat erosion_kernel_orange;
    cv::Mat erosion_kernel_yellow;
    cv::Mat erosion_kernel_magenta;

    cv::Mat dilation_kernel_white;
    cv::Mat dilation_kernel_yellow;

    void ImageCB(const sensor_msgs::ImageConstPtr &msg);

    virtual void onInit();
};

}

