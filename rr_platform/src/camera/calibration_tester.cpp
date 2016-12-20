#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "avc/calibrate_image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>

using namespace std;

void printNextCamHeight(tf::TransformListener& listener) {
    tf::StampedTransform transform;
    listener.waitForTransform("chassis", "camera", ros::Time(0), ros::Duration(10.0));
    listener.lookupTransform("chassis", "camera", ros::Time(0), transform);
    cout << transform.getOrigin().z() << endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "calibration_tester");

    ros::NodeHandle nh;
    ros::Publisher image_pub;
    ros::ServiceClient client = nh.serviceClient<avc::calibrate_image>("/calibrate_image");

    //load image
    //http://answers.ros.org/question/11550/publishing-an-image-from-disk/
    cv_bridge::CvImage cvImage;
    string imagePath("/home/evan/rosbag/calibration_image.jpg");
    cvImage.image = cv::imread(imagePath, CV_LOAD_IMAGE_COLOR);
    cvImage.encoding = "bgr8";
    sensor_msgs::Image rosImage;
    cvImage.toImageMsg(rosImage);
    
    avc::calibrate_image srv;
    srv.request.image = rosImage;
    srv.request.chessboardRows = 7;
    srv.request.chessboardCols = 9;
    srv.request.squareWidth = 0.02745;

    tf::TransformListener listener;

    ros::Duration(1.0).sleep();

    printNextCamHeight(listener);

    if(client.call(srv))
        cout << "calibration service call success" << endl;
    else
        cout << "calibration service call failure" << endl;

    ros::Duration(3.0).sleep();

    printNextCamHeight(listener);

    return 0;
}
