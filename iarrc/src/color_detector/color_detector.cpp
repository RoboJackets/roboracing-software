#include "color_detector.h"
#include <pluginlib/class_list_macros.h>

using namespace std;
using namespace ros;
using namespace cv;

namespace iarrc {

    const Scalar blue_low{78, 50, 70};
    const Scalar blue_high{138, 255, 255};
    const Scalar blue_label{255, 0, 0};

    const Scalar white_low{0, 00, 180}; 
    const Scalar white_high{255, 35, 255};
    const Scalar white_label{255, 255, 255};

    const Scalar orange_low{0, 80, 40};
    const Scalar orange_high{35, 255, 255};
    const Scalar orange_label{0, 127, 255};

    const Scalar yellow_low{25, 60, 45}; //55,50 -> 65, 23->33
    const Scalar yellow_high{45, 255, 255}; //47 > 57
    const Scalar yellow_label{0, 255, 255};
    
    const Scalar magenta_low{165, 25, 66}; 
    const Scalar magenta_high{255, 255, 255}; 
    const Scalar magenta_label{255 , 0, 0};

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
        GaussianBlur(frameBGR, frameBlurred, Size{7,7}, 0);
        Mat frameHSV;
        cvtColor(frameBlurred, frameHSV, CV_BGR2HSV);

        const Mat frame_masked = frameHSV(mask);

        Mat output_blue = Mat::zeros(mask.height, mask.width, CV_8U);
        Mat output_white = Mat::zeros(mask.height, mask.width, CV_8U);
        Mat output_orange = Mat::zeros(mask.height, mask.width, CV_8U);
        Mat output_yellow = Mat::zeros(mask.height, mask.width, CV_8U);
        Mat output_magenta = Mat::zeros(mask.height, mask.width, CV_8U);

        inRange(frame_masked, blue_low, blue_high, output_blue);
        inRange(frame_masked, white_low, white_high, output_white);
        inRange(frame_masked, orange_low, orange_high, output_orange);
        inRange(frame_masked, yellow_low, yellow_high, output_yellow);
        inRange(frame_masked, magenta_low, magenta_high, output_magenta);

        erode(output_blue, output_blue, erosion_kernel_blue);
        erode(output_white, output_white, erosion_kernel_white);
        erode(output_orange, output_orange, erosion_kernel_orange);
        erode(output_yellow, output_yellow, erosion_kernel_yellow);
        erode(output_magenta, output_magenta, erosion_kernel_magenta);

        dilate(output_white, output_white, dilation_kernel_white);
        dilate(output_yellow, output_yellow, dilation_kernel_yellow);

        Mat output = Mat::zeros(frameHSV.rows, frameHSV.cols, CV_8UC3);
        Mat output_masked = output(mask);

        output_masked.setTo(yellow_label, output_yellow);
        output_masked.setTo(orange_label, output_orange);
        output_masked.setTo(white_label, output_white);
        output_masked.setTo(blue_label, output_blue);
        output_masked.setTo(magenta_label, output_magenta);

        img_pub.publish(cv_bridge::CvImage{std_msgs::Header(), "bgr8", output}.toImageMsg());
    }

    void color_detector::onInit() {
        NodeHandle nh = getNodeHandle();
        NodeHandle pnh = getPrivateNodeHandle();
        image_transport::ImageTransport it(nh);

        int mask_y_coordinate;
        pnh.param("mask_y_coordinate", mask_y_coordinate, 280);

        mask = Rect(0, mask_y_coordinate, 640, 480-mask_y_coordinate); // x, y, w, h

        erosion_kernel_blue = getStructuringElement(MORPH_ELLIPSE, Size(11, 11));
        erosion_kernel_white = getStructuringElement(MORPH_ELLIPSE, Size(9, 9));
        erosion_kernel_orange = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
        erosion_kernel_yellow = getStructuringElement(MORPH_ELLIPSE, Size(7, 7)); 
        erosion_kernel_magenta = getStructuringElement(MORPH_ELLIPSE, Size(7, 7)); 
        erosion_kernel_yellow = getStructuringElement(MORPH_ELLIPSE, Size(9, 9));

        dilation_kernel_white = getStructuringElement(MORPH_ELLIPSE, Size(5,5));
        dilation_kernel_yellow = getStructuringElement(MORPH_ELLIPSE, Size(5,5));

        img_sub = it.subscribe("/camera/image_rect", 1, &color_detector::ImageCB, this);
        img_pub = it.advertise("/colors_img", 1);

        ROS_INFO("Color Detector ready!");
    }

}

PLUGINLIB_EXPORT_CLASS(iarrc::color_detector, nodelet::Nodelet)
