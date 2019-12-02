#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <opencv2/core/mat.hpp>

#include <rr_msgs/ColorFilterConfig.h>

namespace rr {

class ColorFilter {
  public:
    explicit ColorFilter(ros::NodeHandle nh);

    cv::Mat Filter(const cv::Mat& img);

  private:
    enum Mode { PASSTHROUGH = 0, GRAY = 1, HSV = 2, HLS = 3 };

    void ReconfigureCallback(const rr_msgs::ColorFilterConfig& config, uint32_t level);

    cv::Vec3i lower_;
    cv::Vec3i upper_;
    int mode_;
    int dilation_;
    int erosion_;
    cv::Rect roi_;

    std::unique_ptr<dynamic_reconfigure::Server<rr_msgs::ColorFilterConfig>> dsrv_;
};

}  // namespace rr
