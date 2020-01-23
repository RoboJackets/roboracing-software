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

    inline bool IsConfigured() const {
        return configured_;
    }

  private:
    enum Mode { SAME_AS_INPUT = 0, GRAY = 1, HSV = 2, HLS = 3 };

    void ReconfigureCallback(const rr_msgs::ColorFilterConfig& config, uint32_t level);

    // Reconfigurable arguments
    cv::Vec3i lower_;
    cv::Vec3i upper_;
    int color_space_;
    int dilation_;
    int erosion_;
    int proc_width_;
    int proc_height_;
    int return_width_;
    int return_height_;
    int hood_mask_width_;
    int hood_mask_height_;
    cv::Rect roi_;
    bool return_roi_only_;

    bool configured_;
    std::unique_ptr<dynamic_reconfigure::Server<rr_msgs::ColorFilterConfig>> dsrv_;
    ros::Publisher debug_pub_;
};

}  // namespace rr
