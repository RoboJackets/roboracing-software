#include <ros/ros.h>
#include <ros/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

using namespace std;

ros::Publisher pub_line_detector;

cv::Mat kernel(int x, int y) {
    return cv::getStructuringElement(cv::MORPH_RECT,cv::Size(x,y));
}

cv::Mat getColHist(cv::Mat img) {
    cv::Mat1i pixels(img.cols, 1);
    for(int x = 0; x < img.cols; x++){
      cv::Mat col = img.col(x);
      pixels(x,0) = cv::countNonZero(col);
    }
    return pixels;
}

//re-center around the average x coordinate of the line segment
cv::Point centerOnLineSegment(cv::Mat imGray, cv::Point currCenter, cv::Point offset, int width, int height) {
    cv::Point topLeft = currCenter - offset;
    int sumX = 0;
    int count = 0;
    for (int y = topLeft.y; y < topLeft.y + height; y++) {
        for (int x = topLeft.x; x < topLeft.x + width; x++) {
            if ((int)imGray.at<unsigned char>(y,x) > 0) {
                sumX += x;
                count++;
            }
        }
    }
    if (count != 0) { //#TODO: set a minimum threshold of pixels
        //we found a line segment
        int avgX = sumX / count;
        currCenter.x = avgX;
    }

    return currCenter;

}


/**
 * Use histogram to find start of lines, then use sliding window to track line
 */
void img_callback(const sensor_msgs::ImageConstPtr& msg) {
    //Convert msg to Mat image
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    cv::Mat frame = cv_ptr->image;
    cv::Mat output;
    cv::cvtColor(frame, output, cv::COLOR_GRAY2BGR); //Doing this just for debugging

    //locate beginnings of lines by a large number of pixels
    cv::Mat hist = getColHist(frame);
    //left line
    double max;
    double min;
    cv::Point leftMaxLoc;
    cv::Point leftMinLoc;
    cv::Point rightMaxLoc;
    cv::Point rightMinLoc;
    cv::minMaxLoc(hist(cv::Range(0,hist.rows/2 - 1), cv::Range::all()), &min, &max, &leftMinLoc, &leftMaxLoc);
    leftMaxLoc.x = leftMaxLoc.y; //gotta flip x and y
    leftMaxLoc.y = frame.rows - 1;
    cv::minMaxLoc(hist(cv::Range(hist.rows/2, hist.rows - 1), cv::Range::all()), &min, &max, &rightMinLoc, &rightMaxLoc);
    rightMaxLoc.x = rightMaxLoc.y + hist.rows/2;
    rightMaxLoc.y = frame.rows - 1;



    cv::circle(output, leftMaxLoc, 8, cv::Scalar(0,0,255), -1);
    cv::circle(output, rightMaxLoc, 8, cv::Scalar(0,255,255), -1);


    int width = 40;
    int height = 16;

    cv::Point offset(width/2, height/2);
    cv::Point center = rightMaxLoc;
    center.y = frame.rows - height/2;

    while (center.y >= 0) {
        center = centerOnLineSegment(frame, center, offset, width, height);
        cv::circle(output, center, 2, cv::Scalar(0, 0, 255), -1);
        cv::Point topLeft = center - offset;
        cv::rectangle(output, topLeft, center + offset, cv::Scalar(0,255,0), 1);
        center.y -= height;


        //shrink the search width as we are more sure of our line
        //#TODO: this is in progress. Could build out from "center" clump we find.
        if (width > 11) {
            width -= 6;
            offset.x = width/2;
        }
    }



/*
    int width = 40;
    int height = 16;

    cv::Point offset(width/2, height/2);
    cv::Point center = rightMaxLoc;
    center.y = frame.rows - height/2;//frame.rows - 1;
    for (int i = frame.rows - 1; i >= 0; i -= height) {
        center = recenter(frame, center, offset);//findNewCenter(frame, center, offset);

        cv::rectangle(output, center - offset, center + offset, cv::Scalar(0,255,0), 2);
        center -= cv::Point(0, height);
    }
*/



    //Publish Message
    if (pub_line_detector.getNumSubscribers() > 0) {
        sensor_msgs::Image outmsg;
        cv_ptr->image = output;
        cv_ptr->encoding = "bgr8";
        cv_ptr->toImageMsg(outmsg);
        pub_line_detector.publish(outmsg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "line_maker");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string subscription_node;
    //nhp.param("perfect_lines_min_cut", perfect_lines_min_cut, 200);


    nhp.param("subscription_node", subscription_node, std::string("/lines/detection_img_transformed"));

    pub_line_detector = nh.advertise<sensor_msgs::Image>("/line_track", 1); //test publish of image
    auto img_real = nh.subscribe(subscription_node, 1, img_callback);

    ros::spin();
    return 0;
}
