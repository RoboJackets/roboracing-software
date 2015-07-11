#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <iarrc/constants.hpp>

#include "projection.h"

using namespace cv_bridge;

ros::Publisher proj_pub;

proj::ProjectionParams *params = NULL;
void imageCallback(const sensor_msgs::Image::ConstPtr& image) {
    CvImageConstPtr cv_ptr;
    cv_ptr = toCvShare(image, "bgr8");
    cv::Mat proj;
    proj::groundTransformProj(cv_ptr->image, *params, proj);

    std_msgs::Header header;
    CvImage projImage(header, "bgr8", proj);

    proj_pub.publish(projImage.toImageMsg());
}

#define DECLARE_PARAM(name, type, defVal)\
    type name;\
    if (!n.getParam(#name, name)) {\
        name = defVal;\
  }

proj::ProjectionParams getParams() {

    ros::NodeHandle n("~");
    DECLARE_PARAM(output_x_res, int, 800);
    DECLARE_PARAM(output_y_res, int, 800);
    DECLARE_PARAM(ground_x_dim, double, 2.03);
    DECLARE_PARAM(ground_y_dim, double, 2.50);
    DECLARE_PARAM(ground_z_dim, double, 0.381);
    DECLARE_PARAM(camera_scale, double, 200.0);
    DECLARE_PARAM(camera_pitch, double, M_PI_2);

    return proj::ProjectionParams(output_x_res,output_y_res,ground_x_dim,ground_y_dim,ground_z_dim,camera_scale,camera_pitch);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "projection");
    ros::NodeHandle n;

    proj::ProjectionParams localParams = getParams();
    params = &localParams;
    ros::Subscriber sub = n.subscribe<sensor_msgs::Image>("image_raw", 1, imageCallback);
    proj_pub = n.advertise<sensor_msgs::Image>("image_projected", 100);

    ros::spin();
    return 0;
}