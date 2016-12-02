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

//temporary constants
const int input_width = 1920;
const int input_height = 1080;
const double fov_h = 0.5932; //angle from center of image to left or right edge
const double fov_v = 0.3337; //angle from center of iamge to rop or bottom edge
const double cam_height = 0.3; //camera height in meters
const double dist_min = 0.75; //minimum distance forward from cam to show in map
const double dist_max = 5.0; //maximum distance (both meters)
const double cam_mount_angle = 0.145; //angle of camera from horizontal
//output image size can be redefined, as in the geometry transform function

int map_pixels_per_meter = 100;

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

bool CalibrateGeometryFromImage(avc::calibrate_image::Request &request, avc::calibrate_image::Response &response) {
    cv_bridge::CvImagePtr cv_ptr;
    Mat outimage;
    cv_ptr = cv_bridge::toCvCopy(request.image);
    const Mat &inimage = cv_ptr->image;

    vector<Point2f> corners;

    bool patternfound = findChessboardCorners(inimage, 
            Size(request.chessboardDim[0], request.chessboardDim[1]),
            corners, 
            CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

    ROS_INFO("Process corners");

    if (!patternfound) {
        ROS_WARN("Pattern not found!");
        return false;
    }

    Mat gray;
    cvtColor(inimage, gray, CV_BGR2GRAY);

    //hone in on the precise corners by identifying edge lines in sourrounding +-11 pixel box
    cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), 
                 TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    //Real world chessboard dimensions
    double chessboardMetersH = request.squareWidth * (request.chessboardEdgesH - 1);
    double chessboardMetersV = request.squareWidth * (request.chessboardEdgesV - 1);



    //Point2f src[4] = {corners[0], corners[8], corners[54], corners[62]};
    return true;
}

/*
 * 
 * refactor dist2px algorithms and their inverses to here
 *
 *
 */

//https://drive.google.com/file/d/0Bw7-7Y3CUDw1Z0ZqdmdRZ3dUTE0/view?usp=sharing
void setTransformFromGeometry() {
    //set width and height of the rectangle in front of the robot with 1cm = 1px
    // the actual output image will show more than this rectangle
    int rectangle_w = sqrt(dist_min*dist_min + cam_height*cam_height) 
                      * tan(fov_h) * map_pixels_per_meter * 2;
    int rectangle_h = (dist_max - dist_min) * map_pixels_per_meter;

    //calculate half the width of the top of the transform box, in input image pixels
    // min_hyp, max_hyp, and theta1 are just temporary variables
    // see https://www.desmos.com/calculator/jsofcq1bi5
    double min_hyp = sqrt(dist_min*dist_min + cam_height*cam_height);
    double max_hyp = sqrt(dist_max*dist_max + cam_height*cam_height);
    double theta1 = atan((min_hyp/max_hyp)*tan(fov_h));
    double x_top_spread = 0.5*input_width*(theta1/fov_h);

    //calculate the top and bottom y coords from the specified distances
    // see https://www.desmos.com/calculator/pwjwlnnx77
    double partial_btm = atan(cam_height/dist_min);
    double partial_top = atan(cam_height/dist_max);
    double y_bottom = input_height*(partial_btm-cam_mount_angle+fov_v)/(2*fov_v);
    double y_top    = input_height*(partial_top-cam_mount_angle+fov_v)/(2*fov_v);

    //set the ouput image size to include the whole transformed image,
    // not just the target rectangle
    out_img_width = rectangle_w * (input_width/x_top_spread) / 2;
    out_img_height = rectangle_h;

    Point2f src[4] = {
        Point2f(input_width/2 - x_top_spread, y_top), //top left
        Point2f(input_width/2 + x_top_spread, y_top), //top right
        Point2f(0, y_bottom),                         //bottom left
        Point2f(input_width, y_bottom)                //bottom right
    };

    Point2f dst[4] = {
        Point2f(out_img_width/2 - rectangle_w/2, 0),              //top left
        Point2f(out_img_width/2 + rectangle_w/2, 0),              //top right
        Point2f(out_img_width/2 - rectangle_w/2, out_img_height), //bottom left
        Point2f(out_img_width/2 + rectangle_w/2, out_img_height)  //bottom right
    };

    transform_matrix = getPerspectiveTransform(src, dst);
}


int main(int argc, char **argv) {

    //namedWindow("Image Window", WINDOW_NORMAL);

    init(argc, argv, "image_transform");
    NodeHandle nh;
    //NodeHandle nh_private("~");

    //transform_matrix = (Mat)(Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

    //transform_file = nh_private.param("transform_file", string("transform.yaml"));
    //loadTransformFromFile(transform_file);


    /*TODO load geometry from tf system*/

    setTransformFromGeometry();

    ServiceServer transform_service = nh.advertiseService("transform_image", TransformImage);
    ServiceServer calibrate_service = nh.advertiseService("calibrate_image", CalibrateGeometryFromImage);

    spin();

    return 0;
}
