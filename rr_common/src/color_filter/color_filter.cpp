#include "rr_common/color_filter.h"

#include <opencv2/opencv.hpp>

namespace rr {

ColorFilter::ColorFilter(ros::NodeHandle nh)
      : lower_(0, 0, 0), upper_(255, 255, 255), mode_(PASSTHROUGH), dilation_(0), erosion_(0), configured_(false) {
    dsrv_ = std::make_unique<dynamic_reconfigure::Server<rr_msgs::ColorFilterConfig>>(nh);
    dsrv_->setCallback(boost::bind(&ColorFilter::ReconfigureCallback, this, _1, _2));

    ROS_INFO_STREAM("Initialized ColorFilter at " << nh.getNamespace());
}

cv::Mat ColorFilter::Filter(const cv::Mat& in_img_full) {
    if (!configured_) {
        ROS_WARN("[ColorFilter] Running Filter before configuration callback");
    }

    roi_.x = std::clamp(roi_.x, 0, in_img_full.cols - 1);
    roi_.y = std::clamp(roi_.y, 0, in_img_full.rows - 1);
    roi_.width = std::clamp(roi_.width, 1, in_img_full.cols - roi_.x);
    roi_.height = std::clamp(roi_.height, 1, in_img_full.rows - roi_.y);

    const cv::Mat in_img = in_img_full(roi_);
    cv::Mat out_img;

    if (mode_ == PASSTHROUGH) {
        if (in_img.channels() == 3) {
            cv::inRange(in_img, lower_, upper_, out_img);
        } else if (in_img.channels() == 1) {
            cv::inRange(in_img, cv::Vec<int, 1>(lower_[0]), cv::Vec<int, 1>(upper_[0]), out_img);
        } else {
            ROS_WARN("[ColorFilter] Passthrough mode, unsupported number of channels %d (should be 1 or 3)",
                     (int)in_img.channels());
        }
    } else if (mode_ == GRAY) {
        cv::Mat cvt_img;
        cv::cvtColor(in_img, cvt_img, CV_BGR2GRAY);
        cv::inRange(cvt_img, cv::Vec<int, 1>(lower_[0]), cv::Vec<int, 1>(upper_[0]), out_img);
    } else if (mode_ == HSV) {
        cv::Mat cvt_img;
        cv::cvtColor(in_img, cvt_img, CV_BGR2HSV);
        cv::inRange(cvt_img, lower_, upper_, out_img);
    } else if (mode_ == HLS) {
        cv::Mat cvt_img;
        cv::cvtColor(in_img, cvt_img, CV_BGR2HLS);
        cv::inRange(cvt_img, lower_, upper_, out_img);
    }

    if (dilation_ > 0) {
        auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size2i(dilation_ * 2 + 1, dilation_ * 2 + 1));
        cv::morphologyEx(out_img, out_img, cv::MORPH_DILATE, kernel);
    }
    if (erosion_ > 0) {
        auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size2i(erosion_ * 2 + 1, erosion_ * 2 + 1));
        cv::morphologyEx(out_img, out_img, cv::MORPH_ERODE, kernel);
    }

    cv::Mat out_img_full;
    if (in_img.size == in_img_full.size) {
        out_img_full = out_img;
    } else {
        out_img_full = cv::Mat(in_img_full.rows, in_img_full.cols, out_img.depth(), cv::Scalar(0, 0, 0));
        out_img.copyTo(out_img_full(roi_));
    }
    return out_img_full;
}

void ColorFilter::ReconfigureCallback(const rr_msgs::ColorFilterConfig& config, uint32_t level) {
    lower_ = cv::Vec3i(config.min1, config.min2, config.min3);
    upper_ = cv::Vec3i(config.max1, config.max2, config.max3);

    int width = std::max(0, config.roi_col_max - config.roi_col_min);
    int height = std::max(0, config.roi_row_max - config.roi_row_min);
    roi_ = cv::Rect(config.roi_col_min, config.roi_row_min, width, height);

    mode_ = config.mode;

    dilation_ = config.dilation;
    erosion_ = config.erosion;

    ROS_INFO("[ColorFilter] reconfigure callback");
    configured_ = true;
}

}  // namespace rr
