#include "color_detector.h"
#include <pluginlib/class_list_macros.h>

using namespace std;
using namespace ros;
using namespace cv;

namespace iarrc {

    const Scalar magenta_low{165, 25, 66}; 
    const Scalar magenta_high{255, 255, 255}; 
    const Scalar magenta_label{255 , 0, 0};

    const Scalar magenta_low_2{0, 25, 66}; 
    const Scalar magenta_high_2{15, 255, 255}; 
    const Scalar magenta_label_2{255 , 0, 0};

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
        GaussianBlur(frameBGR, frameBlurred, Size{0,0}, 7);
        Mat frameHSV;
        cvtColor(frameBlurred, frameHSV, CV_BGR2HSV);

        const Mat frame_masked = frameHSV(mask);

        Mat output_magenta = Mat::zeros(mask.height, mask.width, CV_8U);
        Mat output_magenta_2 = Mat::zeros(mask.height, mask.width, CV_8U);

        inRange(frame_masked, magenta_low, magenta_high, output_magenta);
        inRange(frame_masked, magenta_low_2, magenta_high_2, output_magenta_2);

        erode(output_magenta, output_magenta, erosion_kernel_magenta);
        erode(output_magenta_2, output_magenta_2, erosion_kernel_magenta);

        Mat output = Mat::zeros(frameHSV.rows, frameHSV.cols, CV_8UC3);
        Mat output_masked = output(mask);

        output_masked.setTo(magenta_label, output_magenta);
        output_masked.setTo(magenta_label_2, output_magenta_2);

        img_pub.publish(cv_bridge::CvImage{std_msgs::Header(), "bgr8", output}.toImageMsg());
    }

    void color_detector::onInit() {
        NodeHandle nh = getNodeHandle();
        NodeHandle pnh = getPrivateNodeHandle();
        image_transport::ImageTransport it(nh);

        mask = Rect(0, 482, 1280, 482); // x, y, w, h

        erosion_kernel_blue = getStructuringElement(MORPH_ELLIPSE, Size(11, 11));
        erosion_kernel_white = getStructuringElement(MORPH_ELLIPSE, Size(9, 9));
        erosion_kernel_orange = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
        erosion_kernel_yellow = getStructuringElement(MORPH_ELLIPSE, Size(7, 7)); 
        erosion_kernel_magenta = getStructuringElement(MORPH_ELLIPSE, Size(7, 7)); 
        erosion_kernel_yellow = getStructuringElement(MORPH_ELLIPSE, Size(9, 9));

        dilation_kernel_white = getStructuringElement(MORPH_ELLIPSE, Size(5,5));
        dilation_kernel_yellow = getStructuringElement(MORPH_ELLIPSE, Size(5,5));

        img_sub = it.subscribe("/camera/image_color_rect_flipped", 1, &color_detector::ImageCB, this);
        img_pub = it.advertise("/colors_img", 1);

        ROS_INFO("Color Detector ready!");
    }

}

PLUGINLIB_EXPORT_CLASS(iarrc::color_detector, nodelet::Nodelet)
