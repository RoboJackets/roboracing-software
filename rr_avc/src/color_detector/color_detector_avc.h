#include <cv_bridge/cv_bridge.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rr_platform/transform_image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

ros::Publisher img_pub;
cv::Mat mask;

std::vector<cv::Scalar> lows;
std::vector<cv::Scalar> highs;

cv::Mat detectObstacleColor(const cv::Mat& image, const cv::Scalar& low, const cv::Scalar& high);
void ImageRectCB(const sensor_msgs::ImageConstPtr& msg);

std::string image_topic;
std::string pub_topic;
int image_width;
int image_height;
int mask_px_top;
int mask_px_bottom;
int blur_strength;
