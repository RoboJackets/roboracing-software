#include "start_detector.h"
#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace ros;
using namespace cv;

namespace avc {
    //const Scalar red_low{0, 0, 100};
    //const Scalar red_high{100, 100, 255};
    
    Scalar red_low;
    Scalar red_high;

    void start_detector::ImageCB(const sensor_msgs::ImageConstPtr &msg) {
        cv_bridge::CvImageConstPtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("CV-Bridge error: %s", e.what());
            return;
        }
        
        const Mat &frameBGR = cv_ptr->image;
	Mat frameHSV;
	cvtColor(frameBGR, frameHSV, CV_BGR2HSV);
        Mat blurredImage;
        GaussianBlur(frameHSV, blurredImage, Size{21, 21}, 15);
        Mat redMask = Mat::zeros(blurredImage.rows, blurredImage.cols, CV_8U);
        inRange(blurredImage, red_low, red_high, redMask);
        Mat redImage;
        blurredImage.copyTo(redImage, redMask);
        Mat gray;
        cvtColor(redImage, gray, CV_BGR2GRAY);
	threshold(gray, gray, 10, 255, 0);
        vector<Vec3f> circles;
        HoughCircles(gray, circles, CV_HOUGH_GRADIENT, dp, minDist, param1, param2, minSize, maxSize);
	Mat render;
	cvtColor(gray, render, CV_GRAY2BGR);
	for (auto i = 0; i < circles.size(); i++) {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	    circle(render, center, circles[i][2], Scalar(255, 0, 0), 3);
	}
        if (circles.size() != 0) {
            int maxSum = 0, maxX = 0, maxY = 0, maxR = 0;
            for (size_t i = 0; i < circles.size(); i++) {
                Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                Mat circleMask = Mat::zeros(gray.rows, gray.cols, CV_8U);
                circle(circleMask, center, circles[i][2], Scalar(255, 255, 255), -1); 
                Mat circleImage;
                gray.copyTo(circleImage, circleMask);
                int imgSum = countNonZero(circleImage);
		if (imgSum > maxSum) {
		    maxX = circles[i][0];
		    maxY = circles[i][1];
		    maxR = circles[i][2];
		}
                maxSum = max(maxSum, imgSum);
            }
	    auto areaPercentage = static_cast<double>(maxSum) / (3.14*maxR*maxR);
	    detection_ring_buffer.push_back(areaPercentage);
	    detection_ring_buffer.erase(detection_ring_buffer.begin());
	    auto avgDetections = static_cast<double>(std::accumulate(detection_ring_buffer.begin(), detection_ring_buffer.end(), 0.0)) / static_cast<double>(detection_ring_buffer.size());
            if (avgDetections > 0.8) {
                std_msgs::Bool start;
                start.data = true;
                start_pub.publish(start); 
	        circle(render, Point(maxX, maxY), maxR, Scalar(0, 255, 0), 3);
            } else {
		std_msgs::Bool start;
                start.data = false;
                start_pub.publish(start);
	    }
        } else {
		std_msgs::Bool start;
		start.data = false;
		start_pub.publish(start); 
	    }
        imshow("hough", render);
	waitKey(10);
    }

    void start_detector::onInit() {
        NodeHandle nh = getNodeHandle();
        NodeHandle pnh = getPrivateNodeHandle();
        image_transport::ImageTransport it(nh);

        pnh.param("red_low_r", red_low_r, 100.0);
        pnh.param("red_low_g", red_low_g, 0.0);
        pnh.param("red_low_b", red_low_b, 0.0);
        red_low = Scalar{red_low_b, red_low_g, red_low_r};

        pnh.param("red_high_r", red_high_r, 255.0);
        pnh.param("red_high_g", red_high_g, 100.0);
        pnh.param("red_high_b", red_high_b, 100.0);
        red_high = Scalar{red_high_b, red_high_g, red_high_r};

        pnh.param("dp", dp, 1);
        pnh.param("minDist", minDist, 150);
        pnh.param("param1", param1, 10);
        pnh.param("param2", param2, 25);
        pnh.param("minSize", minSize, 50);
        pnh.param("maxSize", maxSize, 150);

        pnh.param("sumThreshold", sumThreshold, 10000);
        
        img_sub = it.subscribe("/camera_wide/image_raw", 1, &start_detector::ImageCB, this);
        start_pub = nh.advertise<std_msgs::Bool>("/start_detected", 1);

	detection_ring_buffer.resize(10);

        ROS_INFO("Start Detector Ready!");
    }

}

PLUGINLIB_EXPORT_CLASS(avc::start_detector, nodelet::Nodelet)
