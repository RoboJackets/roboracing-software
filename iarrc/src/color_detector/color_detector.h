#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

class color_detector {
private:

    void ImageCB(const sensor_msgs::ImageConstPtr &msg);

    virtual void onInit();
};

