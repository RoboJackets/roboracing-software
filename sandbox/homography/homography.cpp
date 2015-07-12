#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char** argv) {
  VideoCapture cap(1);
  if (!cap.isOpened()) {
    std::cout << "Could not open input video source: " << 1 << std::endl;
    return -1;
  }

  Mat frame;

  cap >> frame;
 
  std::vector<cv::Point2f> corners;
  findChessboardCorners(frame, Size(8, 6), corners);

  namedWindow("Frame");
  imshow("Frame", frame);
  cvWaitKey();

  std::vector<Point2f> src, dest;
  src.push_back(corners.at(0));
  src.push_back(corners.at(7));
  src.push_back(corners.at(40));
  src.push_back(corners.at(47));

  dest.push_back(Point2f(391, 687));
  dest.push_back(Point2f(409, 687));
  dest.push_back(Point2f(391, 700));
  dest.push_back(Point2f(409, 700));

  std::cout << corners.size() << std::endl;
  Mat H = getPerspectiveTransform(src, dest);
  std::cout << H << std::endl;

  Mat transform;
  warpPerspective(frame, transform, H, Size(800, 800));

  imshow("Frame", transform);
  cvWaitKey();

}
