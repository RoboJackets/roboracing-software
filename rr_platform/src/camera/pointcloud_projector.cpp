#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "CameraGeometry.h"

rr::CameraGeometry cam_geom;
ros::Publisher pointcloud_pub;

void image_cb(const sensor_msgs::ImageConstPtr& msg) {
  const cv::Mat& cv_img = cv_bridge::toCvCopy(msg, "mono8")->image;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  for (int r = 0; r < cv_img.rows; r++) {
    for (int c = 0; c < cv_img.cols; c++) {
      if (cv_img.at<uint8_t>(r, c) > 0) {
        auto [in_front, point] = cam_geom.ProjectToWorld(r, c);
        if (in_front) {
          cloud.push_back(pcl::PointXYZ(point.x, point.y, 0));
        }
      }
    }
  }

  sensor_msgs::PointCloud2 out;
  pcl::toROSMsg(cloud, out);
  out.header.frame_id = "base_footprint";
  pointcloud_pub.publish(out);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "image_transform_test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string camera_info_topic;
  std::string camera_tf_frame;
  std::string image_topic_in;
  std::string pointcloud_topic_out;
  double load_timeout;

  bool all_defined = true;
  all_defined &= nh_private.getParam("camera_info_topic", camera_info_topic);
  all_defined &= nh_private.getParam("camera_tf_frame", camera_tf_frame);
  all_defined &= nh_private.getParam("image_topic_in", image_topic_in);
  all_defined &= nh_private.getParam("pointcloud_topic_out", pointcloud_topic_out);
  all_defined &= nh_private.getParam("load_timeout", load_timeout);

  if (!all_defined) {
    ROS_WARN("Pointcloud projector is missing roslaunch params");
  }

  // load camera geometry
  cam_geom.LoadInfo(nh, camera_info_topic, camera_tf_frame, load_timeout);

  auto sub = nh.subscribe(image_topic_in, 1, image_cb);

  pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/test_cloud", 1);

  ros::spin();

  return 0;
}
