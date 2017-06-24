#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

using namespace std;
using namespace ros;
using namespace cv;

namespace iarrc {
    class color_detector: public nodelet::Nodelet {

        private:
            Publisher img_pub;
            Rect mask;
            Mat erosion_kernel;
            void ImageCB(const sensor_msgs::ImageConstPtr &msg);
            inline bool is_blue(const uchar &H, const uchar &S);
            inline bool is_orange(const uchar &H, const uchar &S, const uchar &V);
            inline bool is_yellow(const uchar &H, const uchar &S, const uchar &V);
            inline bool is_white(const uchar &H, const uchar &S, const uchar &V);
            virtual void onInit();


    };

}

