#include "color_detector.h"
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Header.h>


using namespace std;
using namespace ros;
using namespace cv;

namespace iarrc {

using uchar = unsigned char;

inline bool color_detector::is_blue(const uchar &H, const uchar &S, const uchar &V) {
  return (abs(H - 108) < 30) && (S > 50) && (V > 70);
}

inline bool color_detector::is_orange(const uchar &H, const uchar &S, const uchar &V) {
  return (abs(H - 15) < 14) && (S > 70) && (V > 40);
}

inline bool color_detector::is_yellow(const uchar &H, const uchar &S, const uchar &V) {
  return (abs(H - 40) < 7) && (S > 65) && (V > 65);
}

inline bool color_detector::is_white(const uchar &S, const uchar &V) {
  return (S < 62 && S > 25) && (V > 180);
}

void color_detector::ImageCB(const sensor_msgs::ImageConstPtr &msg) {

  cv_bridge::CvImageConstPtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("CV-Bridge error: %s", e.what());
    return;
  }

  const Mat &frameBGR = cv_ptr->image;
  const Mat frameHSV = Mat::zeros(frameBGR.rows, frameBGR.cols, CV_8UC3);
  cvtColor(frameBGR, frameHSV, CV_BGR2HSV);
  const Mat frame_masked = frameHSV(mask);
  Mat output_white = Mat::zeros(frameHSV.rows, frameHSV.cols, CV_8UC1);
  Mat output_white_masked = output_white(mask);
  Mat output_orange = Mat::zeros(frameHSV.rows, frameHSV.cols, CV_8UC1);
  Mat output_orange_masked = output_white(mask);
  Mat output_blue = Mat::zeros(frameHSV.rows, frameHSV.cols, CV_8UC1);
  Mat output_blue_masked = output_white(mask);
  Mat output_yellow = Mat::zeros(frameHSV.rows, frameHSV.cols, CV_8UC1);
  Mat output_yellow_masked = output_white(mask);

  for (int r = 0; r < frame_masked.rows; r++) {
    const uchar *row = frame_masked.ptr<uchar>(r);
    uchar *out_row_white = output_white_masked.ptr<uchar>(r);
    uchar *out_row_orange = output_orange_masked.ptr<uchar>(r);
    uchar *out_row_blue = output_blue_masked.ptr<uchar>(r);
    uchar *out_row_yellow = output_yellow_masked.ptr<uchar>(r);
    for (int c = 0; c < frame_masked.cols; c++) {
      const uchar &H = row[c * 3];
      const uchar &S = row[c * 3 + 1];
      const uchar &V = row[c * 3 + 2];

      if (is_orange(H, S, V)) {
        out_row_orange[c] = 255;
      } else if (is_yellow(H, S, V)) {
        out_row_yellow[c] = 255;
      } else if (is_blue(H, S, V)) {
        out_row_blue[c] = 255;
      } else if (is_white(S, V)) {
        out_row_white[c] = 255;
      }
    }
  }

//        erode(output_white_masked, output_white_masked, erosion_kernel);

  Mat output;

  img_pub.publish(cv_bridge::CvImage{std_msgs::Header(), "bgr8", output}.toImageMsg());
}

void color_detector::onInit() {
  ROS_INFO("spinning up color detector");
  NodeHandle nh = getNodeHandle();
  image_transport::ImageTransport it(nh);

  mask = Rect(0, 200, 640, 280); // x, y, w, h

  auto kernel_size = 5;
  erosion_kernel = getStructuringElement(MORPH_CROSS, Size(kernel_size, kernel_size));

  image_transport::Subscriber img_saver_sub = it.subscribe("/camera/image_rect", 1, &color_detector::ImageCB, this);
  img_pub = it.advertise("/colors_img", 1);
  spin();

}

}

PLUGINLIB_EXPORT_CLASS(iarrc::color_detector, nodelet::Nodelet)
