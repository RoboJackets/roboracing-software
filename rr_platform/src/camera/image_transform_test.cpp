#include <ros/ros.h>
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
  const auto cv_img = cv_bridge::toCvCopy(msg, "mono8")->image;
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

  // load camera geometry
  cam_geom.LoadInfo(nh, "/camera/camera_info", "camera", 60.0);

  ROS_INFO("finished load info");

  auto sub = nh.subscribe("/lines_detection_img", 1, image_cb);

  pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/test_cloud", 1);

  ros::spin();

  return 0;
}
