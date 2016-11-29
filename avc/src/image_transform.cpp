#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "avc/transform_image.h"
#include "avc/calibrate_image.h"
#include <urdf_model/model.h>
#include <cmath>

using namespace std;
using namespace cv;
using namespace ros;

const int input_width = 1920;
const int input_height = 1080;
const double fov_h = 0.5932; //angle from center of image to left or right edge
const double fov_v = 0.3337; //angle from center of iamge to rop or bottom edge
const double cam_height = 0.3; //camera height in meters
const double dist_min = 0.75; //minimum distance forward from cam to show in map
const double dist_max = 5.0; //maximum distance (both meters)
const double cam_mount_angle = 0.155; //TODO establish actual value
//set map width and height with 1cm = 1px
const int map_width = sqrt(dist_min*dist_min + cam_height*cam_height) * tan(fov_h)*200;
const int map_height = (dist_max - dist_min) * 100;
const int out_img_width = input_width;
const int out_img_height = input_height;

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

    warpPerspective(inimage, outimage, transform_matrix, Size(out_img_width, out_img_height));

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


void setGeometryCalibration() {
    double min_hyp = sqrt(dist_min*dist_min + cam_height*cam_height);
    double max_hyp = sqrt(dist_max*dist_max + cam_height*cam_height);
    double theta1 = atan((min_hyp/max_hyp)*tan(fov_h));
    double x_top_spread = 0.5*input_width*(theta1/fov_h + 1) - input_width/2;

    double partial_btm = atan(cam_height/dist_min);
    double partial_top = atan(cam_height/dist_max);
    double y_bottom = input_height*(partial_btm-cam_mount_angle+fov_v)/(2*fov_v);
    double y_top    = input_height*(partial_top-cam_mount_angle+fov_v)/(2*fov_v);

    Point2f src[4] = {
        Point2f(input_width/2 - x_top_spread, y_top), //top left
        Point2f(input_width/2 + x_top_spread, y_top), //top right
        Point2f(0, y_bottom), //bottom left
        Point2f(input_width, y_bottom) //bottom right
    };

    Point2f dst[4] = {
        Point2f(out_img_width/2-map_width/2, out_img_height/2-map_height/2), //top left
        Point2f(out_img_width/2+map_width/2, out_img_height/2-map_height/2), //top right
        Point2f(out_img_width/2-map_width/2, out_img_height/2+map_height/2), //bottom left
        Point2f(out_img_width/2+map_width/2, out_img_height/2+map_height/2) //bottom right
    };

    transform_matrix = getPerspectiveTransform(src, dst);
}


int main(int argc, char **argv) {

    //namedWindow("Image Window", WINDOW_NORMAL);

    init(argc, argv, "image_transform");
    NodeHandle nh;
    NodeHandle nh_private("~");

    transform_matrix = (Mat)(Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

    //transform_file = nh_private.param("transform_file", string("transform.yaml"));
    //loadTransformFromFile(transform_file);

    setGeometryCalibration();

    ServiceServer transform_service = nh.advertiseService("transform_image", TransformImage);
    ServiceServer calibrate_service = nh.advertiseService("calibrate_image", CalibrateImage);

    spin();

    return 0;
}
