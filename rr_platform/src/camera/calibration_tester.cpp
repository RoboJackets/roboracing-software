#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "avc/calibrate_image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "calibration_tester");

    ros::NodeHandle nh;
    ros::Publisher image_pub;
    ros::ServiceClient client = nh.serviceClient<avc::calibrate_image>("/calibrate_image");

    //load image
    //http://answers.ros.org/question/11550/publishing-an-image-from-disk/
    cv_bridge::CvImage cvImage;
    cvImage.image = cv::imread("calibration_image.jpg");
    cvImage.encoding = "bgr8";
    sensor_msgs::Image rosImage;
    cvImage.toImageMsg(rosImage);
    
    avc::calibrate_image srv;
    srv.request.image = rosImage;
    srv.request.chessboardRows = 7;
    srv.request.chessboardCols = 9;
    srv.request.squareWidth = 0.033;

    if(client.call(srv))
        cout << "calibration service call success" << endl;
    else
        cout << "calibration service call failure" << endl;

    //TODO add testing code
}
