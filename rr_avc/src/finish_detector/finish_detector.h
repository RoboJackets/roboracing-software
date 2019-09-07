#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int8.h>
#include <opencv2/highgui.hpp>

namespace rr_avc {
class finish_detector : public nodelet::Nodelet {
private:
  ros::Publisher crosses_pub;
  ros::Publisher debug_pub;
  image_transport::Subscriber img_saver_sub;

  void ImageCB(const sensor_msgs::ImageConstPtr &msg);
  virtual void onInit();

  static constexpr int HIGH = 1;
  static constexpr int LOW = 0;

  float red_low_h;
  float red_low_s;
  float red_low_v;
  float red_high_h;
  float red_high_s;
  float red_high_v;
  cv::Scalar red_low1;
  cv::Scalar red_low2;
  cv::Scalar red_high1;
  cv::Scalar red_high2;

  int state;
  int number_of_crosses;
  ros::Time lastCross;
};

}  // namespace rr_avc
