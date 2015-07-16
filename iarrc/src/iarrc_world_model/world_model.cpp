#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

#include <iarrc/constants.hpp>

using namespace cv_bridge;
using namespace sensor_msgs;

ros::Publisher world_pub;
boost::shared_ptr<CvImage> obstacleImgPtr;
boost::shared_ptr<CvImage> cameraImagePtr;


void bigCB() {
    if (!obstacleImgPtr.get() || !cameraImagePtr.get()) {
        return;
    }
    cv::Mat cameraCopy;
    cv::copyMakeBorder(cameraImagePtr.get()->image, cameraCopy, 
        constants::image_pixel_delta, 
        constants::image_size - constants::image_pixel_delta, 
        0, 0, cv::BORDER_CONSTANT, 128);

    cv::Mat weighted;
    cv::addWeighted(obstacleImgPtr.get()->image, .5, cameraCopy, .5, 0, weighted);
    world_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", weighted).toImageMsg());
}

void obstacleCB(const sensor_msgs::Image::ConstPtr& msg) {
    obstacleImgPtr = toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    bigCB();
}

void cameraCB(const sensor_msgs::Image::ConstPtr& msg) {
    cameraImagePtr = toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    bigCB();
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "world_model");
    std::string obstacle_topic, projection_topic;
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    nhp.param(std::string("obstacle_topic"), obstacle_topic, std::string("/obstacle_img"));
    nhp.param(std::string("projection_topic"), projection_topic, std::string("/image_lines"));

    ros::Subscriber obstacle_sub = nh.subscribe(obstacle_topic, 1, obstacleCB);
    ros::Subscriber projection_sub = nh.subscribe(projection_topic, 1, cameraCB);
    
    world_pub = nh.advertise<sensor_msgs::Image>("/world_model", 1);
    ros::spin();
}
