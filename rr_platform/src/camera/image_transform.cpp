#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "avc/transform_image.h"
#include "avc/calibrate_image.h"
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <avc/constants.hpp>

using namespace std;
using namespace cv;
using namespace ros;

//constants
const double fov_h = constants::camera_fov_horizontal; //angle from center of image to left or right edge
const double fov_v = constants::camera_fov_vertical; //angle from center of image to rop or bottom edge
const double dist_min = constants::camera_distance_min; //minimum distance forward from cam to show in map
const double dist_max = constants::camera_distance_max; //maximum distance (both meters)

double cam_mount_angle;//angle of camera from horizontal
int map_width; //pixels = cm
int map_height;
double map_pixels_per_meter;
int input_width;
int input_height;
double cam_height;//camera height in meters

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
        ROS_INFO_STREAM("Could not find transform at " << file 
                        << ". Will generate new transform.");
        fs.release();
        saveTransformToFile(file);
    }
}

bool TransformImage(avc::transform_image::Request &request, 
                    avc::transform_image::Response &response) {
    cv_bridge::CvImagePtr cv_ptr;
    Mat outimage;
    cv_ptr = cv_bridge::toCvCopy(request.image);
    const Mat &inimage = cv_ptr->image;

    warpPerspective(inimage, outimage, transform_matrix, Size(map_width, map_height));

    cv_ptr->image = outimage;
    cv_ptr->toImageMsg(response.image);

    return true;
}

/*
 * precondition: outPoints is array of length 4 and type Point2f
 * postcondition: outPoints contains pixel coords in topLeft, topRight, bottomLeft,
 *  bottomRight order.
 */
bool getCalibBoardCorners(const Mat &inimage, Size dims, Point2f * outPoints) {
    vector<Point2f> corners;
    int prefs = CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK;
    bool patternfound = findChessboardCorners(inimage, dims, corners, prefs);

    ROS_INFO("Process corners");

    if (!patternfound) {
        ROS_WARN("Pattern not found!");
        return false;
    }

    Mat gray;
    cvtColor(inimage, gray, CV_BGR2GRAY);

    //home in on the precise corners using edge lines in sourrounding +-11 pixel box
    cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), 
                 TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    outPoints[0] = corners[0];
    outPoints[1] = corners[dims.width - 1];
    outPoints[2] = corners[dims.width * (dims.height-1)];
    outPoints[3] = corners[dims.width * dims.height - 1];
}

//update the tf system with the current info
void broadcastCameraTf() {
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    tf::Vector3 cameraLocation(0, 0, cam_height);
    transform.setOrigin(cameraLocation);

    tf::Quaternion cameraRotation;
    cameraRotation.setRPY(0, cam_mount_angle * -1, 0); //angle down from horizontal
    transform.setRotation(cameraRotation);

    tf::StampedTransform st(transform, ros::Time::now(), "chassis", "camera");
    br.sendTransform(st);
}

//inputSize is in pixels
//corners is topLeft, topRight, bottomLeft, bottomRight
void setGeometry(Size boardMeters, Size inputSize, Point2f * corners) {
    Point2f topLeft = corners[0];
    Point2f topRight = corners[1];
    Point2f bottomLeft = corners[2];
    Point2f bottomRight = corners[3];

    // image angle: angle between center of image and bottom (closest) edge of board
    double yBottom = (bottomLeft.y + bottomRight.y) / 2.0;
    double imageAngle = ((2. * yBottom / inputSize.height) - 1) * fov_h;

    // dist0: 3D distance from camera to front edge of board
    double bottomAngleH = (bottomRight.x - bottomLeft.x) * fov_h / 2;
    double dist0 = boardMeters.width / (2. * tan(bottomAngleH));

    // phi1: camera view angle between bottom/front and top/back of board
    double yTop = (topLeft.y + topRight.y) / 2.0;
    double phi1 = (yBottom - yTop) * fov_v;
    // phi2: angle between ground plane, back edge of board, and camera
    double phi2 = asin(dist0 / boardMeters.height * sin(phi1));
    double bottomAngleV = (M_PI/2) - phi1 - phi2;
    
    cam_mount_angle = (M_PI/2) - bottomAngleV - imageAngle; //*****
    cam_height = dist0 * sin(bottomAngleV); //*****
}

/*
 * Tune the angle and height of the camera using a pattern board
 */
bool CalibrateGeometryFromImage(avc::calibrate_image::Request &request, 
                                avc::calibrate_image::Response &response) 
{
    cv_bridge::CvImagePtr cv_ptr;
    Mat outimage;
    cv_ptr = cv_bridge::toCvCopy(request.image);
    const Mat &inimage = cv_ptr->image;

    //size in pointsPerRow, pointsPerColumn
    Size chessboardVertexDims(request.chessboardCols-1, request.chessboardRows-1);
    
    Point2f corners[4];
    bool foundBoard = getCalibBoardCorners(inimage, chessboardVertexDims, corners);

    if(!foundBoard) return false; // failed to find corners

    //Real world chessboard dimensions. width, height
    Size chessboardMeters(request.squareWidth * request.chessboardCols,
                          request.squareWidth * request.chessboardRows);

    //store input image size
    input_width = inimage.cols;
    input_height = inimage.rows;
    Size imgDims(input_width, input_height);

    setGeometry(chessboardMeters, imgDims, corners);

    broadcastCameraTf();

    return true;
}

/*
 * Start with a horizontal line on the groud at dist_min horizonally in front of the 
 * camera. It fills half the camera's FOV, from the center to the right edge. Then 
 * back up the car so that the line is dist_max away horizontally from the camera. The
 * apparent length of this hypothetical line (in pixels) is the output of this 
 * function. 
 * See https://www.desmos.com/calculator/jsofcq1bi5
 * See https://drive.google.com/file/d/0Bw7-7Y3CUDw1Z0ZqdmdRZ3dUTE0/view?usp=sharing
 * params: dmax - maximum distance to consider
 */
double pxFromDist_X(double dmin, double dmax) {
    // min_hyp, max_hyp, and theta1 are just temporary variables
    double min_hyp = sqrt(dmin*dmin + cam_height*cam_height);
    double max_hyp = sqrt(dmax*dmax + cam_height*cam_height);
    double theta1 = atan((min_hyp/max_hyp)*tan(fov_h));
    return 0.5*input_width*(theta1/fov_h);
}

//calculate the y coord of the input image from the specified distance
// see https://www.desmos.com/calculator/pwjwlnnx77
// see https://drive.google.com/file/d/0Bw7-7Y3CUDw1Z0ZqdmdRZ3dUTE0/view?usp=sharing
double pxFromDist_Y(double dist) {
    double foo = atan(cam_height/dist) - cam_mount_angle + fov_v;
    return input_height * foo / (2*fov_v);
}

void setTransformFromGeometry() {
    //set width and height of the rectangle in front of the robot with 1cm = 1px
    // the actual output image will show more than this rectangle
    int rectangle_w = sqrt(dist_min*dist_min + cam_height*cam_height) 
                      * tan(fov_h) * constants::pixels_per_meter * 2;
    int rectangle_h = (dist_max - dist_min) * constants::pixels_per_meter;

    //find coordinates for corners above rectangle in input image
    double x_top_spread = pxFromDist_X(dist_min, dist_max);
    double y_bottom = pxFromDist_Y(dist_min);
    double y_top    = pxFromDist_Y(dist_max);

    //set the ouput image size to include the whole transformed image,
    // not just the target rectangle
    map_width = rectangle_w * (input_width/x_top_spread) / 2;
    map_height = rectangle_h;

    Point2f src[4] = {
        Point2f(input_width/2 - x_top_spread, y_top), //top left
        Point2f(input_width/2 + x_top_spread, y_top), //top right
        Point2f(0, y_bottom),                         //bottom left
        Point2f(input_width, y_bottom)                //bottom right
    };

    Point2f dst[4] = {
        Point2f(map_width/2 - rectangle_w/2, 0),              //top left
        Point2f(map_width/2 + rectangle_w/2, 0),              //top right
        Point2f(map_width/2 - rectangle_w/2, map_height), //bottom left
        Point2f(map_width/2 + rectangle_w/2, map_height)  //bottom right
    };

    transform_matrix = getPerspectiveTransform(src, dst);
}


int main(int argc, char **argv) {

    //namedWindow("Image Window", WINDOW_NORMAL);

    init(argc, argv, "image_transform");
    NodeHandle nh;

    setTransformFromGeometry();

    ServiceServer transform_service = nh.advertiseService("transform_image", TransformImage);
    ServiceServer calibrate_service = nh.advertiseService("calibrate_image", CalibrateGeometryFromImage);

    spin();

    return 0;
}
