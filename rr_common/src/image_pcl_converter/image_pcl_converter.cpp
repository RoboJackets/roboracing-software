/*
 * Borrows from old color_detector_avc and from image_transform.
 * N.B. pass in a binary image in 1-channel "mono8" format.
 * Any non-zero element in the input image becomes a point in a
 * cloud.
 */

#include <cv_bridge/cv_bridge.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/algorithm/string.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

using uchar = unsigned char;

map<string, ros::Publisher> cloud_pubs;
double pxPerMeter;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

void transformedImageCB(const sensor_msgs::ImageConstPtr& msg, const string& topic) {
  if (cloud_pubs[topic].getNumSubscribers() == 0) {
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("CV_Bridge error: %s", e.what());
    return;
  }

  const cv::Mat& in_image = cv_ptr->image;

  // get outline
  cv::Mat transformed;
  cv::Laplacian(in_image, transformed, CV_16SC1);

  cloud->clear();
  for (int r = 0; r < transformed.rows; r++) {
    auto* row = transformed.ptr<int16_t>(r);
    for (int c = 0; c < transformed.cols; c++) {
      if (row[c] != 0) {
        pcl::PointXYZ point;
        point.y = static_cast<float>(((transformed.cols / 2.0f) - c) / pxPerMeter);
        point.x = static_cast<float>((transformed.rows - r) / pxPerMeter);
        point.z = 0.0;

        cloud->push_back(point);
      }
    }
  }

  pcl::PCLPointCloud2 cloud_pc2;
  pcl::toPCLPointCloud2(*cloud, cloud_pc2);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(cloud_pc2, cloud_msg);
  cloud_msg.header.frame_id = "base_footprint";
  cloud_msg.header.stamp = msg->header.stamp;
  cloud_pubs[topic].publish(cloud_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_pcl_converter");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  string topicsConcat;
  nhp.getParam("image_topics", topicsConcat);
  nhp.getParam("px_per_meter", pxPerMeter);

  cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

  vector<string> topics;
  boost::split(topics, topicsConcat, boost::is_any_of(" ,"));
  vector<ros::Subscriber> image_subs;
  for (const string& topic : topics) {
    if (topic.size() == 0)
      continue;

    auto bound_callback = boost::bind(transformedImageCB, _1, topic);
    auto sub = nh.subscribe<sensor_msgs::Image>(topic, 1, bound_callback);
    image_subs.push_back(sub);

    ROS_INFO_STREAM("image_pcl_converter subscribed to " << topic);
    string newTopic = topic + "_cloud";
    ROS_INFO_STREAM("Creating new topic " << newTopic);
    cloud_pubs[topic] = nh.advertise<sensor_msgs::PointCloud2>(newTopic, 1);
  }

  ros::spin();
  return 0;
}
