#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "rr_platform/calibrate_image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
//#include <rr_avc/constants.hpp>

using namespace std;

tf::TransformListener * listener;

geometry_msgs::Pose testCameraTf() {
    tf::Quaternion q_tf;
    q_tf.setRPY(0,0,0);
    geometry_msgs::Quaternion q_msg;
    tf::quaternionTFToMsg(q_tf, q_msg);

    geometry_msgs::PoseStamped psSrc, psDst;
    psSrc.header.frame_id = "camera";
    psSrc.pose.position.x = 0;
    psSrc.pose.position.y = 0;
    psSrc.pose.position.z = 0;
    psSrc.pose.orientation = q_msg;
    
    listener->waitForTransform("chassis", "camera", ros::Time(0), ros::Duration(1.0));
    listener->transformPose("chassis", psSrc, psDst);
    return psDst.pose;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "calibration_tester");

    ros::NodeHandle nh;
    auto client = nh.serviceClient<rr_platform::calibrate_image>("/calibrate_image");

    //load image
    //http://answers.ros.org/question/11550/publishing-an-image-from-disk/
    cv_bridge::CvImage cvImage;
    string imagePath("/etc/roboracing/calibration_image.png");
    cv::Mat img = cv::imread(imagePath, CV_LOAD_IMAGE_COLOR);
    if(img.empty()) {
        ROS_ERROR("Could not find file %s", imagePath.c_str());
        return 1;
    }
    cvImage.image = img;
    cvImage.encoding = "bgr8";
    sensor_msgs::Image rosImage;
    cvImage.toImageMsg(rosImage);
    
    rr_platform::calibrate_image srv;
    srv.request.image = rosImage;
    srv.request.chessboardRows = 7;
    srv.request.chessboardCols = 9;
    srv.request.squareWidth = 0.02745;

    tf::TransformListener listener_tmp;
    listener = &listener_tmp;

    cout << "camera from ground before: " << endl << testCameraTf() << endl;

    if(client.call(srv))
        cout << "calibration service call success" << endl;
    else
        cout << "calibration service call failure" << endl;

    ros::Duration(0.1).sleep(); //allow tf to update

    cout << "camera from ground tf after: " << endl << testCameraTf() << endl;

    return 0;
}
