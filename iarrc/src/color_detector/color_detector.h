#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace iarrc {
    class color_detector: public nodelet::Nodelet {
        private:
            image_transport::Publisher img_pub;
            cv::Rect mask;
            cv::Mat erosion_kernel;
            void ImageCB(const sensor_msgs::ImageConstPtr &msg);
            inline bool is_blue(const uchar &H, const uchar &S, const uchar &V);
            inline bool is_orange(const uchar &H, const uchar &S, const uchar &V);
            inline bool is_yellow(const uchar &H, const uchar &S, const uchar &V);
            inline bool is_white(const uchar &S, const uchar &V);
            virtual void onInit();
    };

}

