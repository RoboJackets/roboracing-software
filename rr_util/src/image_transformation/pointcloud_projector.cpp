#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.h>
#include <rclcpp/rclcpp.hpp>
#include <rr_util/CameraGeometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <thread>

using PointT = pcl::PointXYZ;

class PointCloudProjector : public nodelet::Nodelet {
  private:
    rr::CameraGeometry cam_geom_;
    ros::Publisher pointcloud_pub_;
    image_transport::Subscriber detection_image_sub_;
    pcl::PointCloud<PointT>::Ptr cloud_unfiltered_;
    pcl::VoxelGrid<PointT> grid_filter_;
    double x_max_;
    int downsample_factor_;
    bool cam_geom_ready_;
    std::thread load_info_thread_;

    void ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
        if (!cam_geom_ready_) {
            NODELET_INFO("Pointcloud projector waiting for camera geometry load");
            return;
        }

        const cv::Mat cv_img = cv_bridge::toCvShare(msg, "mono8")->image;

        cv::Mat cv_img_resized;
        const auto new_width = static_cast<int>(cam_geom_.GetImageWidth() / downsample_factor_);
        const auto new_height = static_cast<int>(cam_geom_.GetImageHeight() / downsample_factor_);
        cv::resize(cv_img, cv_img_resized, cv::Size(new_width, new_height));

        cloud_unfiltered_->clear();
        for (int r = 0; r < cv_img_resized.rows; r++) {
            for (int c = 0; c < cv_img_resized.cols; c++) {
                if (cv_img_resized.at<uint8_t>(r, c) > 0) {
                    auto [in_front, point] = cam_geom_.ProjectToWorld(r * downsample_factor_, c * downsample_factor_);
                    if (!in_front || point.x > x_max_) {
                        break;  // this row is above the horizon or too far away, so skip
                                // the rest of it
                    } else {
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
        out->header.stamp = msg->header.stamp;
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
        all_defined &= nh_private.getParam("max_forward", x_max_);
        all_defined &= nh_private.getParam("downsample_factor", downsample_factor_);

        if (!all_defined) {
            NODELET_WARN("Pointcloud projector is missing roslaunch params");
        }

        // load camera geometry
        cam_geom_ready_ = false;
        load_info_thread_ = std::thread([this, camera_info_topic, camera_tf_frame, load_timeout]() {
            auto nh = getNodeHandle();
            cam_geom_.LoadInfo(nh, camera_info_topic, camera_tf_frame, load_timeout);
            cam_geom_ready_ = true;
        });

        detection_image_sub_ = image_transport.subscribe(image_topic_in, 1, &PointCloudProjector::ImageCallback, this);

        pointcloud_pub_ = node_handle.advertise<sensor_msgs::PointCloud2>(pointcloud_topic_out, 1);

        cloud_unfiltered_.reset(new pcl::PointCloud<PointT>);
        grid_filter_.setLeafSize(0.05f, 0.05f, 0.05f);

        NODELET_INFO("Loaded pointcloud projector nodelet");
    }
};

PLUGINLIB_EXPORT_CLASS(PointCloudProjector, nodelet::Nodelet);
