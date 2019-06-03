#include <stdio.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#define HIST_FRAMES 5     // Number of frames to keep in history


ros::Publisher img_pub;
ros::Publisher bool_pub;
sensor_msgs::Image outmsg;
std_msgs::Bool start_detected;

cv::Mat history[HIST_FRAMES];
int counttostart = 1;

cv::Mat getRedImage(cv::Mat frame) {
    std::vector<cv::Mat> bgr;
    cv::split(frame, bgr);

    cv::Mat redLightCheck = bgr[2] - bgr[0] - bgr[1]; //red - blue - green

    const int thresholdRed = 20;
    cv::threshold(redLightCheck, redLightCheck, thresholdRed, 255, cv::THRESH_BINARY);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
    cv::morphologyEx(redLightCheck, redLightCheck, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(redLightCheck, redLightCheck, cv::MORPH_OPEN, kernel);

    return redLightCheck;
}

cv::Mat getGreenImage(cv::Mat frame) {
    std::vector<cv::Mat> bgr;
    cv::split(frame, bgr);

    cv::Mat greenLightCheck = (bgr[0].mul(bgr[1] - bgr[2]) / 255); //elementwiseMult(blue, green) - red

    const int thresholdGreen = 50;
    cv::threshold(greenLightCheck, greenLightCheck, thresholdGreen, 255, cv::THRESH_BINARY);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
    cv::morphologyEx(greenLightCheck, greenLightCheck, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(greenLightCheck, greenLightCheck, cv::MORPH_OPEN, kernel);

    return greenLightCheck;
}

//@note circles returned is a vector of vectors containing <x, y, radius>
std::vector<cv::Vec3f> findCirclesFromContours(cv::Mat &binary) {
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // Find circles
    double circularityThreshold = 0.8; //based on Hu moments @note: controls the sensitivity of detection
    std::vector<cv::Vec3f> foundCircles;

    for(int i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        double arclength = cv::arcLength(contours[i], true);
        double circularity = 4 * CV_PI * area / (arclength * arclength);
        if (circularity >= circularityThreshold) {
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contours[i], center, radius);
            cv::Vec3f circle{center.x, center.y, radius};
            foundCircles.push_back(circle);
        }
    }
    return foundCircles;
}

std::vector<cv::Vec3f> findCirclesFromHough(cv::Mat &binary) {
    /// Apply the Hough Transform to find the circles
    std::vector<cv::Vec3f> circles;
    double dp = 1;
    double minDist = binary.rows/8;
    double cannyThreshold = 100;
    double accumThreshold = 12;
    int minRadius = 0;
    int maxRadius = binary.rows/3;
    cv::HoughCircles(binary, circles, CV_HOUGH_GRADIENT, dp, minDist, cannyThreshold, accumThreshold, minRadius, maxRadius);
    return circles;
}

//For debugging purposes
void drawCircles(cv::Mat &debug, std::vector<cv::Vec3f> circles) {
    // Draw the circles detected
    for( size_t i = 0; i < circles.size(); i++ ) {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        radius += 0.8 * radius; //add an offset for easier viewing
        // circle center
        cv::circle( debug, center, 3, cv::Scalar(255,255,0), -1, 8, 0 );
        // circle outline
        cv::circle( debug, center, radius, cv::Scalar(0,255,255), 3, 8, 0 );
     }
}





// What happens when the stoplight changes from red to green?
// The stoplight has a red light on from the start
// This light is very bright, and can wash out the image
// It typically looks like a reddish halo around a white circle
// The stoplight then turns on the green light, and both are on
// This looks like two white circles one over the other,
// The top having a red halo, and the bottom a green halo
// (note: the green looks bluish in frame)
// This lasts for a few frames, then the red light turns off

/*
 * The idea: keep checking for green circles, when we find one,
 * go back [x] frames and see if there was a red one above and nearby.
 *
*/
void img_callback(const sensor_msgs::Image::ConstPtr& msg) {

    if(start_detected.data) {
        bool_pub.publish(start_detected);
        return; // do not to all this computing if the signal has already been seen and go broadcast
    }

    //Convert msg to Mat image
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;

    //gather a set of frames to run on
    for(int i = HIST_FRAMES-1; i > 0; i--) {
        history[i] = history[i - 1];
    }
    history[0] = frame;
    if (!(counttostart == HIST_FRAMES)) {
        counttostart++;
        return;
    }

    cv::Mat greenImage = getGreenImage(frame);
    std::vector<cv::Vec3f> greenCircles = findCirclesFromContours(greenImage);


    if (greenCircles.size() > 0) {
        cv::Mat redImage = getRedImage(history[HIST_FRAMES - 1]); //@note only checks farthest history. May want to check more
        std::vector<cv::Vec3f> redCircles = findCirclesFromContours(redImage);
        for (int i = 0; i < greenCircles.size(); i++) {
            cv::Point greenCenter(static_cast<int>(greenCircles[i][0]), static_cast<int>(greenCircles[i][1]));
            double greenRadius = greenCircles[i][2];

            for (int j = 0; j < redCircles.size(); j++) {
                cv::Point redCenter(static_cast<int>(redCircles[j][0]), static_cast<int>(redCircles[j][1]));
                double redRadius = redCircles[j][2];
                const double xTolerance = 20.0; //#TODO: launch params
                const double yTolerance = 100.0;
                const double radiusTolerance = 10.0;
                if (greenCenter.y > redCenter.y && //green below red
                    std::fabs(greenRadius - redRadius) <= radiusTolerance &&
                    std::abs(greenCenter.x - redCenter.x) <= xTolerance &&
                    std::abs(greenCenter.y - redCenter.y) <= yTolerance) {
                    start_detected.data = true; //red light found
                    break;
                }
            }
        }
    }

    bool_pub.publish(start_detected);

    if (img_pub.getNumSubscribers() > 0) {
        //do some debug visualizations
        cv::Mat debug;
        std::vector<cv::Mat> debugChannels(3);
        debugChannels[0] = cv::Mat::zeros(greenImage.rows, greenImage.cols, greenImage.type());
        debugChannels[1] = greenImage;
        debugChannels[2] = getRedImage(history[HIST_FRAMES - 1]); //note: this means what you see is HIST_FRAMES behind
        cv::merge(debugChannels, debug);
        drawCircles(debug, greenCircles);
        drawCircles(debug, findCirclesFromContours(debugChannels[2]));

        cv_ptr->image = debug;
        cv_ptr->encoding = "bgr8";
        cv_ptr->toImageMsg(outmsg);
        img_pub.publish(outmsg);
    }

}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "stoplight_watcher_v2");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    std::string img_topic;
    std::string stoplight_topic;
    nhp.param("img_topic", img_topic, std::string("/camera/image_color_rect"));
    nhp.param("stoplight_watcher_topic", stoplight_topic, std::string("/start_detected"));

    // Subscribe to ROS topic with callback
    start_detected.data = false;
    ros::Subscriber img_saver_sub = nh.subscribe(img_topic, 1, img_callback);
    img_pub = nh.advertise<sensor_msgs::Image>("/stoplight_debug", 1);
    bool_pub = nh.advertise<std_msgs::Bool>(stoplight_topic, 1);

    ros::spin();
    return 0;
}
