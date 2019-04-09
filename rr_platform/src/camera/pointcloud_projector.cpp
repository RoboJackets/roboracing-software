#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <rr_platform/CameraGeometry.h>

using PointT = pcl::PointXYZ;

class PointCloudProjector : public nodelet::Nodelet {
private:
  rr::CameraGeometry cam_geom_;
  ros::Publisher pointcloud_pub_;
  image_transport::Subscriber detection_image_sub_;
  pcl::PointCloud<PointT>::Ptr cloud_unfiltered_;
  pcl::VoxelGrid<PointT> grid_filter_;


  void ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    const cv::Mat cv_img = cv_bridge::toCvShare(msg, "mono8")->image;
    cloud_unfiltered_->clear();

    for (int r = 0; r < cv_img.rows; r++) {
      for (int c = 0; c < cv_img.cols; c++) {
        if (cv_img.at<uint8_t>(r, c) > 0) {
          auto [in_front, point] = cam_geom_.ProjectToWorld(r, c);
          if (!in_front || point.x > 10) {
            break;  // this row is above the horizon or too far away, so skip the rest of it
          } else if (std::abs(point.y) < 10) {
            cloud_unfiltered_->push_back(pcl::PointXYZ(point.x, point.y, 0));
          }
        }
      }
    }

    pcl::PointCloud<PointT> filtered_cloud;
    grid_filter_.setInputCloud(cloud_unfiltered_);
    grid_filter_.filter(filtered_cloud);

    sensor_msgs::PointCloud2Ptr out(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(filtered_cloud, *out);
    out->header.frame_id = "base_footprint";
    pointcloud_pub_.publish(out);
  }

  void onInit() override {
    auto node_handle = getNodeHandle();
    auto nh_private = getPrivateNodeHandle();

    image_transport::ImageTransport image_transport(node_handle);

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
      NODELET_WARN("Pointcloud projector is missing roslaunch params");
    }

    // load camera geometry
    cam_geom_.LoadInfo(node_handle, camera_info_topic, camera_tf_frame, load_timeout);

    detection_image_sub_ = image_transport.subscribe(image_topic_in, 1, &PointCloudProjector::ImageCallback, this);

    pointcloud_pub_ = node_handle.advertise<sensor_msgs::PointCloud2>(pointcloud_topic_out, 1);

    cloud_unfiltered_.reset(new pcl::PointCloud<PointT>);
    grid_filter_.setLeafSize(0.05f, 0.05f, 0.05f);

    NODELET_INFO("Loaded pointcloud projector nodelet");
  }
};

PLUGINLIB_EXPORT_CLASS(PointCloudProjector, nodelet::Nodelet);
