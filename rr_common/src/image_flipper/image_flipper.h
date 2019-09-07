#ifndef PROJECT_IMAGEFLIPPERNODELET_H
#define PROJECT_IMAGEFLIPPERNODELET_H

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int8.h>
#include <opencv2/highgui.hpp>

namespace rr_common {
class image_flipper : public nodelet::Nodelet {
private:
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  void onInit() override;

  image_transport::Subscriber img_sub;
  image_transport::Publisher img_pub;

  int flip_code;
  cv_bridge::CvImage output;
};
}  // namespace rr_common

#endif  // PROJECT_IMAGEFLIPPERNODELET_H
