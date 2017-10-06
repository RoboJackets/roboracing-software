#include "finish_detector.h"
#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>

using namespace std;
using namespace ros;
using namespace cv;

namespace avc {

    void finish_detector::ImageCB(const sensor_msgs::ImageConstPtr &msg) {
        cv_bridge::CvImageConstPtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("CV-Bridge error: %s", e.what());
            return;
        }
        
        const Mat &frameBGR = cv_ptr->image;
    }

    void finish_detector::onInit() {
        NodeHandle nh = getNodeHandle();
        NodeHandle pnh = getPrivateNodeHandle();
        image_transport::ImageTransport it(nh);

        //pnh.param("red_low_r", red_low_r, 100.0);
        
        img_sub = it.subscribe("/camera/rgb/image_raw", 1, &finish_detector::ImageCB, this);
        finish_pub = nh.advertise<std_msgs::Bool>("/finish_detected", 1);

        ROS_INFO("Finish Detector Ready!");
    }

}

PLUGINLIB_EXPORT_CLASS(avc::finish_detector, nodelet::Nodelet)
