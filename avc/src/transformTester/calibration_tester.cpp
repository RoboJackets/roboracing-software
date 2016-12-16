#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "avc/calibrate_image.h"



int main(int argc, char** argv) {
    ros::init(argc, argv, "calibration_tester");

    ros::NodeHandle nh;

    ServiceClient client = nh.serviceClient<avc::calibrate_image>("/calibrate_image");

    //load image
    //http://answers.ros.org/question/11550/publishing-an-image-from-disk/?answer=129181#post-id-129181


}
