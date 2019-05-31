#include <ros/package.h>
#include "color_detector.h"
#include <pluginlib/class_list_macros.h>
#include <rr_iarrc/hsv_tuned.h>
#include <fstream>
#include <ctime>

using namespace std;
using namespace ros;
using namespace cv;

namespace rr_iarrc {

    double white_h_low;
    double white_s_low;
    double white_v_low;
    double white_h_high;
    double white_s_high;
    double white_v_high;
    std::string package_path = ros::package::getPath("rr_iarrc");
    std::string load_file_path;

    void color_detector::ImageCB(const sensor_msgs::ImageConstPtr &msg) {

        cv_bridge::CvImageConstPtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("CV-Bridge error: %s", e.what());
            return;
        }

        const Mat &frameBGR = cv_ptr->image;
        Mat frameBlurred;
        GaussianBlur(frameBGR, frameBlurred, Size{0,0}, 1);
        Mat frameHSV;
        cvtColor(frameBlurred, frameHSV, CV_BGR2HSV);

        const Mat frame_masked = frameHSV(mask);

        Mat output_white = Mat::zeros(mask.height, mask.width, CV_8UC1);

        auto white_low = Scalar(white_h_low, white_s_low, white_v_low);
        auto white_high = Scalar(white_h_high, white_s_high, white_v_high);
        inRange(frame_masked, white_low, white_high, output_white);

        erode(output_white, output_white, erosion_kernel_white);
        dilate(output_white, output_white, dilation_kernel_white);

        Mat output = Mat::zeros(frameHSV.rows, frameHSV.cols, CV_8UC1);
        Mat output_masked = output(mask);
        output_masked.setTo(255, output_white);

        img_pub.publish(cv_bridge::CvImage{std_msgs::Header(), "mono8", output}.toImageMsg());
    }

    void hsvTunedCallback(const rr_iarrc::hsv_tuned::ConstPtr &msg){
        white_h_low = msg->white_h_low;
        white_s_low = msg->white_s_low;
        white_v_low = msg->white_v_low;

        white_h_high = msg->white_h_high;
        white_s_high = msg->white_s_high;
        white_v_high = msg->white_v_high;

        ROS_INFO("Set HSV limits");
    }

    void loadValues(){
        string line;
        int value;
        ifstream myfile (load_file_path);

        if (myfile.is_open()){
            ROS_INFO("Loading HSV values.");
            getline(myfile,line);
            value = stoi(line);
            white_h_low = value;

            getline(myfile,line);
            value = stoi(line);
            white_s_low = value;

            getline(myfile,line);
            value = stoi(line);
            white_v_low = value;

            getline(myfile,line);
            value = stoi(line);
            white_h_high = value;

            getline(myfile,line);
            value = stoi(line);
            white_s_high = value;

            getline(myfile,line);
            value = stoi(line);
            white_v_high = value;

            myfile.close();
        }
        else{
            ROS_INFO("Unable to load HSV values.");
        }
    }

    void color_detector::onInit() {
        NodeHandle nh = getNodeHandle();
        NodeHandle pnh = getPrivateNodeHandle();
        image_transport::ImageTransport it(nh);

        std::string camera_image_topic, detection_topic, hsv_values_topic;
        pnh.param(string("camera_image"), camera_image_topic, string("/camera/image_color_rect"));
        pnh.param(string("detection_topic"), detection_topic, string("/hsv_detected"));
        pnh.param(string("hsv_values_topic"), hsv_values_topic, string("/hsv_tuned"));

        ROS_INFO_STREAM("Camera image " << camera_image_topic);

        std::string default_load_file_path = package_path + "/saved_hsv/example.txt" ;
        pnh.param(std::string("load_file"), load_file_path, default_load_file_path);
        loadValues();

        ros::Subscriber hsv_tuned_sub = nh.subscribe(hsv_values_topic, 1, hsvTunedCallback);

        mask = Rect(0, 482, 1280, 482); // x, y, w, h

        erosion_kernel_white = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
        dilation_kernel_white = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));

        img_sub = it.subscribe(camera_image_topic, 1, &color_detector::ImageCB, this);
        img_pub = it.advertise(detection_topic, 1);

        ROS_INFO("Color detector ready!");
        ros::spin();
    }
}

PLUGINLIB_EXPORT_CLASS(rr_iarrc::color_detector, nodelet::Nodelet)
