#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rr_platform/calibrate_image.h>
#include <rr_platform/camera_geometry.h>
#include <cmath>
#include <sensor_msgs/JointState.h>
#include <avc/constants.hpp>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace cv;
using namespace ros;

double camera_fov_horizontal;
double camera_fov_vertical;
double cam_mount_angle; //angle of camera from horizontal
double cam_height; //camera height in meters

Size mapSize; //pixels = cm
Size imageSize;
Mat transform_matrix;

NodeHandle* nh;
Publisher camera_geo_pub;
map<string, Publisher> transform_pubs;

void loadGeometryFromTf() {
    tf::TransformListener listener;

    tf::Quaternion q_tf;
    q_tf.setRPY(0,0,0);
    geometry_msgs::Quaternion q_msg;
    tf::quaternionTFToMsg(q_tf, q_msg);

    geometry_msgs::PoseStamped psSrc, psDst;
    psSrc.header.frame_id = "camera";
    psSrc.pose.position.x = 0;
    psSrc.pose.position.y = 0;
    psSrc.pose.position.z = 0;
    psSrc.pose.orientation = q_msg;

    bool success = false;
    while(!success) {
        try {
            listener.waitForTransform("map", "camera", ros::Time(0), ros::Duration(60.0));
            listener.transformPose("map", psSrc, psDst);
            success = true;
        } catch(tf2::LookupException e) {
            ROS_ERROR("tf LookupException: %s", e.what());
        }
    }

    tf::quaternionMsgToTF(psDst.pose.orientation, q_tf);
    double roll, pitch, yaw;
    tf::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);

    ROS_INFO("found rpy = %f %f %f", roll, pitch, yaw);

    cam_mount_angle = pitch;
    cam_height = psDst.pose.position.z;
}

void fovCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    ROS_INFO("called fovCallback");
    double fx = msg->P[0]; //horizontal focal length of rectified image, in px
    double fy = msg->P[5]; //vertical focal length
    imageSize = Size(msg->width, msg->height);
    camera_fov_horizontal = 2 * atan2(imageSize.width, 2*fx);
    camera_fov_vertical = 2 * atan2(imageSize.height, 2*fy);
}
void loadCameraFOV() {
    camera_fov_horizontal = camera_fov_vertical = -1;
    NodeHandle nh_temp;
    auto infoSub = nh_temp.subscribe("/camera/camera_info", 1, fovCallback);
    while(camera_fov_horizontal <= 0 || camera_fov_vertical <= 0) {
        spinOnce();
        Duration(0.05).sleep();
    }
    nh_temp.shutdown();
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

    //ROS_INFO("Process corners");
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

    return true;
}

// let another (platform-specific) module update the tf system with the current info
void updateJointState() {
    rr_platform::camera_geometry msg;
    msg.height = cam_height;
    msg.angle = cam_mount_angle;
    camera_geo_pub.publish(msg);
}

//corners is topLeft, topRight, bottomLeft, bottomRight
void setGeometry(Size_<double> boardMeters, Point2f * corners) {
    Point2f topLeft = corners[0];
    Point2f topRight = corners[1];
    Point2f bottomLeft = corners[2];
    Point2f bottomRight = corners[3];

    // image angle: angle between center of image and bottom (closest) edge of board
    double yBottom = double(bottomLeft.y + bottomRight.y) / 2;
    double imageAngle = ((yBottom / imageSize.height) - 0.5) * camera_fov_horizontal;

    // dist0: 3D distance from camera to front edge of board
    double bottomAngleH = (bottomRight.x - bottomLeft.x) / imageSize.width * camera_fov_horizontal;
    double dist0 = boardMeters.width / (2. * tan(bottomAngleH));

    // phi1: camera view angle between bottom/front and top/back of board
    double yTop = double(topLeft.y + topRight.y) / 2;
    double phi1 = (yBottom - yTop) / imageSize.height * camera_fov_vertical;
    // phi2: angle between ground plane, back edge of board, and camera
    double phi2 = asin(dist0 / boardMeters.height * sin(phi1));
    double bottomAngleV = (M_PI/2) - phi1 - phi2;
    
    cam_mount_angle = (M_PI/2) - bottomAngleV - imageAngle; //*****
    cam_height = dist0 * sin(bottomAngleV); //*****
}

/*
 * Start with a horizontal line on the groud at dmin meters horizonally in front of the 
 * camera. It fills half the camera's FOV, from the center to the right edge. Then back
 * up the car so that the line is dmax meters away horizontally from the camera. The
 * apparent length of this hypothetical line (in pixels) is the output of this function.
 * See https://www.desmos.com/calculator/jsofcq1bi5
 * See https://drive.google.com/file/d/0Bw7-7Y3CUDw1Z0ZqdmdRZ3dUTE0/view?usp=sharing
 */
double pxFromDist_X(double dmin, double dmax) {
    // min_hyp, max_hyp, and theta1 are just temporary variables
    double min_hyp = sqrt(dmin*dmin + cam_height*cam_height);
    double max_hyp = sqrt(dmax*dmax + cam_height*cam_height);
    double theta1 = atan((min_hyp/max_hyp)*tan(camera_fov_horizontal/2));
    return imageSize.width * (theta1 / camera_fov_horizontal);
}

//calculate the y coord of the input image from the specified distance
// see https://www.desmos.com/calculator/pwjwlnnx77
// see https://drive.google.com/file/d/0Bw7-7Y3CUDw1Z0ZqdmdRZ3dUTE0/view?usp=sharing
double pxFromDist_Y(double dist) {
    double foo = atan(cam_height/dist) - cam_mount_angle + camera_fov_vertical/2;
    return imageSize.height * foo / (camera_fov_vertical);
}

void setTransformFromGeometry() {
    //set width and height of the rectangle in front of the robot with 1cm = 1px
    // the actual output image will show more than this rectangle
    int rectangle_w = sqrt(pow(constants::camera_distance_min,2) + pow(cam_height,2)) 
                      * tan(camera_fov_horizontal/2) * constants::pixels_per_meter * 2;
    int rectangle_h = (constants::camera_distance_max - constants::camera_distance_min) 
                      * constants::pixels_per_meter;

    //find coordinates for corners above rectangle in input image
    double x_top_spread = pxFromDist_X(constants::camera_distance_min, constants::camera_distance_max);
    double y_bottom = pxFromDist_Y(constants::camera_distance_min);
    double y_top    = pxFromDist_Y(constants::camera_distance_max);

    //set the ouput image size to include the whole transformed image,
    // not just the target rectangle
    mapSize.width = rectangle_w * (imageSize.width/x_top_spread) / 2;
    mapSize.height = rectangle_h;

    Point2f src[4] = {
        Point2f(imageSize.width/2 - x_top_spread, y_top), //top left
        Point2f(imageSize.width/2 + x_top_spread, y_top), //top right
        Point2f(0, y_bottom),                             //bottom left
        Point2f(imageSize.width, y_bottom)                //bottom right
    };

    Point2f dst[4] = {
        Point2f(mapSize.width/2 - rectangle_w/2, 0),              //top left
        Point2f(mapSize.width/2 + rectangle_w/2, 0),              //top right
        Point2f(mapSize.width/2 - rectangle_w/2, mapSize.height), //bottom left
        Point2f(mapSize.width/2 + rectangle_w/2, mapSize.height)  //bottom right
    };

    transform_matrix = getPerspectiveTransform(src, dst);
}

void TransformImage(const sensor_msgs::ImageConstPtr msg, string topic) {
    //if no one is listening or the transform is undefined, give up
    if(transform_pubs[topic].getNumSubscribers() == 0 || transform_matrix.empty()) {
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg);
    const Mat &inimage = cv_ptr->image;

    Mat outimage;
    warpPerspective(inimage, outimage, transform_matrix, mapSize);

    sensor_msgs::Image outmsg;
    cv_ptr->image = outimage;
    cv_ptr->toImageMsg(outmsg);
    transform_pubs[topic].publish(outmsg);
}

/*
 * Tune the angle and height of the camera using a pattern board
 */
bool CalibrateCallback(rr_platform::calibrate_image::Request &request, 
                       rr_platform::calibrate_image::Response &response) 
{
    if(camera_fov_horizontal <= 0 || camera_fov_vertical <= 0) {
        ROS_ERROR("Calling calibration callback without a FOV defined");
        return false;
    }

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
    double w = double(request.squareWidth) * double(request.chessboardCols);
    double h = double(request.squareWidth) * double(request.chessboardRows);
    Size_<double> chessboardMeters(w, h);

    setGeometry(chessboardMeters, corners);

    ROS_INFO_STREAM("found height " << cam_height);
    ROS_INFO_STREAM("found angle " << cam_mount_angle);

    updateJointState();

    return true;
}


int main(int argc, char **argv) {
    init(argc, argv, "image_transform");
    NodeHandle nh_temp;
    nh = &nh_temp;

    loadCameraFOV(); //spins ROS event loop for a bit
    loadGeometryFromTf();
    setTransformFromGeometry();
    ROS_INFO("Set transform. Used height %f and angle %f", cam_height, cam_mount_angle);

    //publish camera info for a description module to update its model
    camera_geo_pub = nh->advertise<rr_platform::camera_geometry>("/camera_geometry", 1);

    string topicsConcat;
    nh->getParam("/image_transform/transform_topics", topicsConcat);
    vector<string> topics;
    boost::split(topics, topicsConcat, boost::is_any_of(" ,"));
    vector<Subscriber> transform_subs;
    for(const string& topic : topics) {
        if(topic.size() == 0) continue;
        transform_subs.push_back(nh->subscribe<sensor_msgs::Image>(topic, 1, 
                                 boost::bind(TransformImage, _1, topic)));
        ROS_INFO_STREAM("Image_transform subscribed to " << topic);
        string newTopic(topic + "_transformed");
        ROS_INFO_STREAM("Creating new topic " << newTopic);
        transform_pubs[topic] = nh->advertise<sensor_msgs::Image>(newTopic, 1);
    }

    ServiceServer calibrate_service = nh->advertiseService("/calibrate_image", CalibrateCallback);

    spin();

    return 0;
}
