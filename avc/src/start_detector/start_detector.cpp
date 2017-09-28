#include "start_detector.h"
#include <pluginlib/class_list_macros.h>

using namespace std;
using namespace ros;
using namespace cv;

namespace avc {
    const Scalar red_low{0, 0, 100};
    const Scalar red_high{100, 100, 255};

    void start_detector::ImageCB(const sensor_msgs::ImageConstPtr &msg) {
        cv_bridge::CvImageConstPtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("CV-Bridge error: %s", e.what());
            return;
        }
        
        const Mat &frameBGR = cv_ptr->image;
        Mat blurredImage;
        GaussianBlur(frameBGR, blurredImage, Size{7, 7}, 3);
        Mat mask = Mat::zeros(blurredImage.rows, blurredImage.cols, CV_8U);
        inRange(blurredImage, red_low, red_high, mask);
        Mat output;
        bitwise_and(blurredImage, blurredImage, output, mask);
        Mat gray;
        cvtColor(output, gray, CV_BGR2GRAY);
        vector<Vec3f> circles;
        HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1, 150, 10, 25, 50, 150);
        if (circles.size() != 0) {
            int maxSum = 0, maxX = 0, maxY = 0, maxR = 0;
            for (size_t i = 0; i < circles.size(); i++) {
                Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                Mat circleMask = Mat::zeros(gray.rows, gray.cols, CV_8U);
                circle(circleMask, center, circles[i][2], Scalar(255, 255, 255), -1); 
                Mat outputMasked;
                bitwise_and(output, output, circleMask);
                int imgsum = sum(outputMasked)[0];
                if (imgsum > maxSum) {
                    maxSum = imgsum;
                }
            }

            if (maxSum > 100) {
                std_msgs::Bool start;
                start.data = true;
                start_pub.publish(start); 
            }
            ROS_INFO_STREAM("Max Sum: " << maxSum << "\n");
        }
    }

    void start_detector::onInit() {
        NodeHandle nh = getNodeHandle();
        NodeHandle pnh = getPrivateNodeHandle();
        image_transport::ImageTransport it(nh);

        img_sub = it.subscribe("/camera/image_rect", 1, &start_detector::ImageCB, this);
        start_pub = nh.advertise<std_msgs::Bool>("/start_detected", 1);
    }

}

PLUGINLIB_EXPORT_CLASS(avc::start_detector, nodelet::Nodelet)
