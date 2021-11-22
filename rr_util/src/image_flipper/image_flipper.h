#ifndef PROJECT_IMAGEFLIPPERNODELET_H
#define PROJECT_IMAGEFLIPPERNODELET_H

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.h>
#include <std_msgs/msg/int8.h>

#include <opencv2/highgui.hpp>

namespace rr_util {
class ImageFlipper : public rclcpp::Node {    
  public:
    explicit ImageFlipper(const rclcpp::NodeOptions & options) : rclcpp::Node("image_flipper", options){

    } 
  private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    image_transport::Subscriber img_sub;
    image_transport::Publisher img_pub;

    int flip_code;
    cv_bridge::CvImage output;
};
}  // namespace rr_util

#endif  // PROJECT_IMAGEFLIPPERNODELET_H
