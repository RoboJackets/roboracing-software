#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "avc/transform_image.h"
#include "avc/calibrate_image.h"

using namespace std;
using namespace cv;
using namespace ros;

Mat transform_matrix;
string transform_file;

void saveTransformToFile(std::string file) {
    FileStorage fs(file, FileStorage::WRITE);
    fs << "transform" << transform_matrix;
    fs.release();
}

void loadTransformFromFile(std::string file) {
    FileStorage fs(file, FileStorage::READ);
    if (fs.isOpened()) {
        fs["transform"] >> transform_matrix;
        fs.release();
    } else {
        ROS_INFO_STREAM("Could not find transform at " << file << ". Will generate new transform.");
        fs.release();
        saveTransformToFile(file);
    }
}

bool TransformImage(avc::transform_image::Request &request, avc::transform_image::Response &response) {
    cv_bridge::CvImagePtr cv_ptr;
    Mat outimage;
    cv_ptr = cv_bridge::toCvCopy(request.image);
    const Mat &inimage = cv_ptr->image;

    warpPerspective(inimage, outimage, transform_matrix, Size(1920, 1080));

    cv_ptr->image = outimage;
    cv_ptr->toImageMsg(response.image);

    return true;
}

bool CalibrateImage(avc::calibrate_image::Request &request, avc::calibrate_image::Response &response) {
    cv_bridge::CvImagePtr cv_ptr;
    Mat outimage;
    cv_ptr = cv_bridge::toCvCopy(request.image);
    const Mat &inimage = cv_ptr->image;

    vector<Point2f> corners;

    bool patternfound = findChessboardCorners(inimage, Size(request.chessboardDim[0], request.chessboardDim[1]),
                                              corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
                                                       + CALIB_CB_FAST_CHECK);

    ROS_INFO("Process corners");

    if (!patternfound) {
        ROS_WARN("Pattern not found!");
        return false;
    }

    Mat gray;
    cvtColor(inimage, gray, CV_BGR2GRAY);
    cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    //Real world chessboard dimensions
    double meterDim[2] = {request.squareWidth * (request.chessboardDim[0] - 1),
                          request.squareWidth * (request.chessboardDim[1] - 1)};


    //Convert from meters to pix
    double pixDim[2] = {meterDim[0] * request.pixelsPerMeter,
                        meterDim[1] * request.pixelsPerMeter};

    //Calculate center of destBoard
    int distanceFromBottom = request.distanceToChessboard * request.pixelsPerMeter;
    int yCenter = request.imgDim[0] - distanceFromBottom;
    int xCenter = request.imgDim[1] / 2;
    Point2f center(xCenter, yCenter);

    //populate dst array
    Point2f topLeft = Point2f(-pixDim[0] / 2, -pixDim[1] / 2) + center;
    Point2f topRight = Point2f(pixDim[0] / 2, -pixDim[1] / 2) + center;
    Point2f bottomLeft = Point2f(-pixDim[0] / 2, pixDim[1] / 2) + center;
    Point2f bottomRight = Point2f(pixDim[0] / 2, pixDim[1] / 2) + center;


    Point2f src[4] = {corners[0], corners[8], corners[54], corners[62]};
    Point2f dst[4] = {topLeft, topRight, bottomLeft, bottomRight};
    transform_matrix = getPerspectiveTransform(src, dst);

    saveTransformToFile(transform_file);

    return true;
}

int main(int argc, char **argv) {

    //namedWindow("Image Window", WINDOW_NORMAL);

    init(argc, argv, "image_transform");
    NodeHandle nh;
    NodeHandle nh_private("~");

    transform_matrix = (Mat)(Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

    transform_file = nh_private.param("transform_file", string("transform.yaml"));
    loadTransformFromFile(transform_file);

    ServiceServer transform_service = nh.advertiseService("transform_image", TransformImage);
    ServiceServer calibrate_service = nh.advertiseService("calibrate_image", CalibrateImage);

    spin();

    return 0;
}
