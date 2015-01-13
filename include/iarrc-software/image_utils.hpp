#ifndef IMAGE_UTILS_HPP
#define IMAGE_UTILS_HPP

#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace image_utils {

int persp_transform_width = 2000;
int persp_transform_height = 1500;

cv::Point2f persp_transform_src_pts[4] = {
	cv::Point2f(237,410),
	cv::Point2f(396,410),
	cv::Point2f(426,450),
	cv::Point2f(206,450)
};
cv::Point2f persp_transform_dst_pts[4] = {
	cv::Point2f(-20+(persp_transform_width/2),110),
	cv::Point2f(20+(persp_transform_width/2),110),
	cv::Point2f(20+(persp_transform_width/2),70),
	cv::Point2f(-20+(persp_transform_width/2),70)
};

void transform_perspective(const cv::Mat &input, cv::Mat &output) {
	cv::Mat transform = getPerspectiveTransform(persp_transform_src_pts, persp_transform_dst_pts);
	cv::warpPerspective(input, output, transform, cv::Size(persp_transform_width,persp_transform_height));
}

}

#endif // IMAGE_UTILS_HPP
