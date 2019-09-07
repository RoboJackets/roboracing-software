#ifndef IMAGE_UTILS_HPP
#define IMAGE_UTILS_HPP
#define THRESHOLD_VALUE 5  // 5 - 10 works well

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <vector>

namespace image_utils {
using namespace std;
using namespace cv;

int persp_transform_width = 2000;
int persp_transform_height = 1500;

static vector<Mat> kernal(8);
static vector<Mat> kernalcompl(8);
static float karray[3][9][9] = { { { -1, -1, -1, -1, -1, -1, -1, -1, -1 },
                                   { -1, -1, -1, -1, -1, -1, -1, -1, -1 },
                                   { -1, -1, -1, -1, -1, -1, -1, -1, -1 },
                                   { 1, 1, 1, 1, 1, 1, 1, 1, 1 },
                                   { 1, 1, 1, 1, 1, 1, 1, 1, 1 },
                                   { 1, 1, 1, 1, 1, 1, 1, 1, 1 },
                                   { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
                                   { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
                                   { 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
                                 { { -1, -1, -1, -1, -1, -1, -1, -1, -1 },
                                   { -1, -1, -1, -1, -1, -1, -1, 0, 1 },
                                   { -1, -1, -1, -1, -1, 0, 1, 1, 1 },
                                   { -1, -1, -1, 0, 1, 1, 1, 1, 1 },
                                   { -1, 0, 1, 1, 1, 1, 1, .5, 0 },
                                   { 1, 1, 1, 1, 1, .5, 0, 0, 0 },
                                   { 1, 1, 1, .5, 0, 0, 0, 0, 0 },
                                   { 1, .5, 0, 0, 0, 0, 0, 0, 0 },
                                   { 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
                                 { { -.89, -.89, -.89, -.89, -.89, -.89, -.89, 1, 1 },
                                   { -.89, -.89, -.89, -.89, -.89, -.89, 1, 1, 1 },
                                   { -.89, -.89, -.89, -.89, -.89, 1, 1, 1, 0 },
                                   { -.89, -.89, -.89, -.89, 1, 1, 1, 0, 0 },
                                   { -.89, -.89, -.89, 1, 1, 1, 0, 0, 0 },
                                   { -.89, -.89, 1, 1, 1, 0, 0, 0, 0 },
                                   { -.89, 1, 1, 1, 0, 0, 0, 0, 0 },
                                   { 1, 1, 1, 0, 0, 0, 0, 0, 0 },
                                   { 1, 1, 0, 0, 0, 0, 0, 0, 0 } } };

enum LINES { LINES_NONE, LINES_WHITE, LINES_YELLOW, LINES_BOTH };
vector<Mat> getGeometricMean(Mat& image);
void subtractOrthog(vector<Mat>& images);
Mat combine(vector<Mat>& images);

cv::Point2f persp_transform_src_pts[4] = { cv::Point2f(237, 410), cv::Point2f(396, 410), cv::Point2f(426, 450),
                                           cv::Point2f(206, 450) };
cv::Point2f persp_transform_dst_pts[4] = { cv::Point2f(-20 + (persp_transform_width / 2), 110),
                                           cv::Point2f(20 + (persp_transform_width / 2), 110),
                                           cv::Point2f(20 + (persp_transform_width / 2), 70),
                                           cv::Point2f(-20 + (persp_transform_width / 2), 70) };
void transform_perspective(const cv::Mat& input, cv::Mat& output) {
  cv::Mat transform = getPerspectiveTransform(persp_transform_src_pts, persp_transform_dst_pts);
  cv::warpPerspective(input, output, transform, cv::Size(persp_transform_width, persp_transform_height));
};

LINES hasLines(Mat& image) {
  kernal[0] = Mat(9, 9, CV_32FC1, karray[0]) / 27;
  kernal[1] = Mat(9, 9, CV_32FC1, karray[1]) / 25;
  kernal[2] = Mat(9, 9, CV_32FC1, karray[2]) / 25;

  kernal[3] = kernal[1].t();
  kernal[4] = kernal[0].t();

  flip(kernal[3], kernal[5], 0);
  flip(kernal[2], kernal[6], 0);
  flip(kernal[1], kernal[7], 0);

  // cerr <<" kernalcompl are 180 degree rotations of kernal, looking for the other edge of the line";
  for (int i = 0; i < kernal.size(); i++) {
    kernalcompl[i] = kernal[i].clone();
    flip(kernal[i], kernalcompl[i], -1);
  }
  // cerr << "hellow 0 " << endl;

  Mat channel[3];
  Mat channelLines[3];
  // cerr << "test" << endl;
  split(image, channel);
  for (int i = 0; i < 3; i++) {
    // cerr << "test: " << i << endl;
    // cerr << "channel rows, cols " << channel[i].rows  << ", " << channel[i].cols << endl;
    vector<Mat> results = getGeometricMean(channel[i]);
    // cerr << "test1: " << i << endl;
    subtractOrthog(results);
    // cerr << "test2: " << i << endl;
    Mat finImage = combine(results);
    threshold(finImage, channelLines[i], THRESHOLD_VALUE, 255, CV_THRESH_BINARY);
    // cerr << "test3: " << i << endl;
  }

  Mat whiteLines = ~channelLines[0] & channelLines[1];  // & channelLines[2];
  Mat yellowLines = channelLines[0];
  //    channel[0] = yellowLines + whiteLines;
  //    merge(channel, 3, image);
  // cerr << "test 1" << endl;

  int sumwhite = sum(whiteLines)[0];
  int sumyellow = sum(yellowLines)[0];
  int threshold = 255 * ((image.rows < image.cols ? image.rows : image.cols)) / 2;
  LINES retval = LINES_NONE;
  if (sumwhite > threshold)
    retval = LINES_WHITE;
  if (sumyellow > threshold)
    retval = retval ? LINES_BOTH : LINES_YELLOW;

  // cerr << "test 2" << endl;
  return retval;
}

vector<Mat> getGeometricMean(Mat& image) {
  Mat filtered, filteredcompl;
  vector<Mat> results;
  // cerr << "in geometric mean" << endl;
  // cerr << "kernal.size() :" << kernal.size() << endl;
  // cerr << "kernalcompl.size() :" << kernalcompl.size() << endl;
  for (int i = 0; i < kernal.size(); i++) {
    // cerr << "kernal[" << i << "].size" << kernal[i].size() << endl;
    filter2D(image, filtered, -1, kernal[i]);
    filter2D(image, filteredcompl, -1, kernalcompl[i]);

    filtered.convertTo(filtered, CV_16UC1, 1);
    filteredcompl.convertTo(filteredcompl, CV_16UC1, 1);

    results.push_back(filtered.mul(filteredcompl));
    results[i].convertTo(results[i], CV_8UC1, 1.0 / 256);
  }

  return results;
}

void subtractOrthog(vector<Mat>& images) {
  vector<Mat> imagesCopy;
  for (Mat& img : images)
    imagesCopy.push_back(img.clone());
  for (int i = 0; i < images.size(); i++) {
    images[i] -= imagesCopy[(i + images.size() / 2) % images.size()];
  }
}

Mat combine(vector<Mat>& images) {
  Mat result = images[0].clone();
  for (int i = 1; i < images.size(); i++) {
    result = max(result, images[i]);
  }

  return result;
}
}  // namespace image_utils

#endif  // IMAGE_UTILS_HPP
