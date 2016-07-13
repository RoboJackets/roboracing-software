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

//  cap >> frame;
 
  std::vector<cv::Point2f> corners;
 // findChessboardCorners(frame, Size(8, 6), corners);

//  namedWindow("Frame");
//  imshow("Frame", frame);
//  cvWaitKey();

//  imwrite("newdump.jpg", frame);
  std::vector<Point2f> src, dest;
  src.push_back(Point2f(177, 91));
  src.push_back(Point2f(410, 98));
  src.push_back(Point2f(64, 124));
  src.push_back(Point2f(516, 138));

  dest.push_back(Point2f(278.08, 217.8));
  dest.push_back(Point2f(278.08+243.84, 217.8));
  dest.push_back(Point2f(278.08, 217.8+304.8));
  dest.push_back(Point2f(278.08+243.84, 217.8+304.8));

  std::cout << corners.size() << std::endl;
  Mat H = getPerspectiveTransform(src, dest);
  std::cout << H << std::endl;

  Mat transform;
  warpPerspective(frame, transform, H, Size(800, 800));

//  imshow("Frame", transform);
//  cvWaitKey();

}
