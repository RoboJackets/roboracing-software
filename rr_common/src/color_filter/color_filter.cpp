#include "rr_common/color_filter.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

namespace rr {

ColorFilter::ColorFilter(ros::NodeHandle nh)
      : lower_(0, 0, 0)
      , upper_(255, 255, 255)
      , color_space_(SAME_AS_INPUT)
      , dilation_(0)
      , erosion_(0)
      , proc_width_(0)
      , proc_height_(0)
      , return_width_(0)
      , return_height_(0)
      , hood_mask_width_(0)
      , hood_mask_height_(0)
      , return_roi_only_(false)
      , configured_(false) {
    dsrv_ = std::make_unique<dynamic_reconfigure::Server<rr_msgs::ColorFilterConfig>>(nh);
    dsrv_->setCallback(boost::bind(&ColorFilter::ReconfigureCallback, this, _1, _2));

    debug_pub_ = nh.advertise<sensor_msgs::Image>("debug_img", 1);

    ROS_INFO_STREAM("Initialized ColorFilter at " << nh.getNamespace());
}

cv::Mat ColorFilter::Filter(const cv::Mat& in_img_full) {
    if (!configured_) {
        ROS_WARN("[ColorFilter] Running Filter before configuration callback");
    }

    // Step 1: resize to processing size
    cv::Mat resized_img;
    if (proc_width_ > 0 && proc_height_ > 0) {
        cv::resize(in_img_full, resized_img, cv::Size(proc_width_, proc_height_));
    } else {
        resized_img = in_img_full;  // shares by reference, no copy
    }

    // Step 2: get region of interest and mask out hood
    roi_.x = std::clamp(roi_.x, 0, resized_img.cols - 1);
    roi_.y = std::clamp(roi_.y, 0, resized_img.rows - 1);
    roi_.width = std::clamp(roi_.width, 1, resized_img.cols - roi_.x);
    roi_.height = std::clamp(roi_.height, 1, resized_img.rows - roi_.y);

    cv::Mat input_resized_roi(roi_.height, roi_.width, resized_img.depth());
    resized_img(roi_).copyTo(input_resized_roi);

    if (hood_mask_width_ > 0 && hood_mask_height_ > 0) {
        cv::Rect hood_rect;
        hood_rect.x = (input_resized_roi.cols - hood_mask_width_) / 2;
        hood_rect.y = input_resized_roi.rows - hood_mask_height_;
        hood_rect.height = hood_mask_height_;
        hood_rect.width = hood_mask_width_;
        cv::rectangle(input_resized_roi, hood_rect, cv::Scalar(0, 0, 0), -1);
    }

    // Step 3: convert colorspace and mask by in-range check
    cv::Mat processed_roi;

    if (color_space_ == SAME_AS_INPUT) {
        if (input_resized_roi.channels() == 3) {
            cv::inRange(input_resized_roi, lower_, upper_, processed_roi);
        } else if (input_resized_roi.channels() == 1) {
            cv::inRange(input_resized_roi, cv::Vec<int, 1>(lower_[0]), cv::Vec<int, 1>(upper_[0]), processed_roi);
        } else {
            ROS_WARN("[ColorFilter] Passthrough mode, unsupported number of channels %d (should be 1 or 3)",
                     (int)input_resized_roi.channels());
        }
    } else if (color_space_ == GRAY) {
        cv::Mat cvt_img;
        cv::cvtColor(input_resized_roi, cvt_img, CV_BGR2GRAY);
        cv::inRange(cvt_img, cv::Vec<int, 1>(lower_[0]), cv::Vec<int, 1>(upper_[0]), processed_roi);
    } else if (color_space_ == HSV) {
        cv::Mat cvt_img;
        cv::cvtColor(input_resized_roi, cvt_img, CV_BGR2HSV);
        cv::inRange(cvt_img, lower_, upper_, processed_roi);
    } else if (color_space_ == HLS) {
        cv::Mat cvt_img;
        cv::cvtColor(input_resized_roi, cvt_img, CV_BGR2HLS);
        cv::inRange(cvt_img, lower_, upper_, processed_roi);
    }

    // Step 4: apply dilation and erosion
    if (dilation_ > 0) {
        auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size2i(dilation_ * 2 + 1, dilation_ * 2 + 1));
        cv::morphologyEx(processed_roi, processed_roi, cv::MORPH_DILATE, kernel);
    }
    if (erosion_ > 0) {
        auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size2i(erosion_ * 2 + 1, erosion_ * 2 + 1));
        cv::morphologyEx(processed_roi, processed_roi, cv::MORPH_ERODE, kernel);
    }

    // Step 5: copy ROI back into resized full frame, if desired
    cv::Mat padded_img;
    if (return_roi_only_) {
        padded_img = processed_roi;
    } else {
        padded_img = cv::Mat(resized_img.rows, resized_img.cols, processed_roi.depth(), cv::Scalar(0, 0, 0));
        processed_roi.copyTo(padded_img(roi_));
    }

    // Step 6: resize to final size
    cv::Mat return_img;
    if (return_width_ == 0 || return_height_ == 0 ||
        (padded_img.cols == return_width_ && padded_img.rows == return_height_)) {
        // return width matches current size or is unspecified. No resize
        return_img = padded_img;
    } else {  // resize to return size
        cv::resize(padded_img, return_img, cv::Size(return_width_, return_height_));
    }

    if (debug_pub_.getNumSubscribers() > 0) {
        cv_bridge::CvImage cvb;
        cvb.image = return_img;
        cvb.encoding = "mono8";
        debug_pub_.publish(cvb.toImageMsg());
    }

    return return_img;
}

void ColorFilter::ReconfigureCallback(const rr_msgs::ColorFilterConfig& config, uint32_t level) {
    lower_ = cv::Vec3i(config.min1, config.min2, config.min3);
    upper_ = cv::Vec3i(config.max1, config.max2, config.max3);

    int width = std::max(0, config.roi_col_max - config.roi_col_min);
    int height = std::max(0, config.roi_row_max - config.roi_row_min);
    roi_ = cv::Rect(config.roi_col_min, config.roi_row_min, width, height);

    color_space_ = config.mode;

    dilation_ = config.dilation;
    erosion_ = config.erosion;

    return_roi_only_ = config.return_roi_only;

    proc_width_ = config.proc_width;
    proc_height_ = config.proc_height;
    return_width_ = config.return_width;
    return_height_ = config.return_height;

    hood_mask_height_ = config.roi_hood_mask_height;
    hood_mask_width_ = config.roi_hood_mask_width;

    ROS_INFO("[ColorFilter] reconfigure callback");
    configured_ = true;
}

}  // namespace rr
