#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <rr_platform/CameraGeometry.h>
#include <sensor_msgs/Image.h>
#include <boost/algorithm/string.hpp>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace ros;

double px_per_meter;           // resolution of overhead view
double camera_dist_max;        // distance to look ahead of the car
double camera_dist_min;        // avoid the front bumper
double camera_fov_horizontal;  // radians
double camera_fov_vertical;
double cam_mount_angle;   // angle of camera from horizontal
double cam_mount_height;  // camera height from ground in meters
double cam_mount_x;       // distance from camera to base_footprint

Size mapSize;  // pixels = cm
Size imageSize;
Mat transform_matrix;

map<string, Publisher> transform_pubs;

/*
 * Start with a horizontal line on the groud at dmin meters horizonally in front
 * of the camera. It fills half the camera's FOV, from the center to the right
 * edge. Then back up the car so that the line is dmax meters away horizontally
 * from the camera. The apparent length of this hypothetical line (in pixels) is
 * the output of this function. See https://www.desmos.com/calculator/jsofcq1bi5
 * See
 * https://drive.google.com/file/d/0Bw7-7Y3CUDw1Z0ZqdmdRZ3dUTE0/view?usp=sharing
 */
double pxFromDist_X(double dmin, double dmax) {
    double min_hyp = sqrt(dmin * dmin + cam_mount_height * cam_mount_height);
    double max_hyp = sqrt(dmax * dmax + cam_mount_height * cam_mount_height);
    double theta1 = atan((min_hyp / max_hyp) * tan(camera_fov_horizontal / 2));
    return imageSize.width * (theta1 / camera_fov_horizontal);
}

// calculate the y coord of the input image from the specified distance
// see https://www.desmos.com/calculator/pwjwlnnx77
double pxFromDist_Y(double dist) {
    double tmp = atan(cam_mount_height / dist) - cam_mount_angle + camera_fov_vertical / 2;
    return imageSize.height * tmp / (camera_fov_vertical);
}

void setTransformFromGeometry() {
    // set width and height of the rectangle in front of the robot
    // the actual output image will show more than this rectangle
    float close_corner_dist = sqrt(pow(camera_dist_min, 2) + pow(cam_mount_height, 2));
    float rectangle_w = close_corner_dist * tan(camera_fov_horizontal / 2) * px_per_meter * 2;
    float rectangle_h = (camera_dist_max - camera_dist_min) * px_per_meter;

    // find coordinates for corners above rectangle in input image
    float x_top_spread = pxFromDist_X(camera_dist_min, camera_dist_max);
    float y_bottom = pxFromDist_Y(camera_dist_min);
    float y_top = pxFromDist_Y(camera_dist_max);

    // set the ouput image size to include the whole transformed image,
    // not just the target rectangle
    mapSize.width = static_cast<int>(rectangle_w * (imageSize.width / x_top_spread) / 2.0);
    mapSize.height = static_cast<int>(rectangle_h);

    Point2f src[4] = {
        Point2f(imageSize.width / 2.f - x_top_spread, y_top),  // top left
        Point2f(imageSize.width / 2.f + x_top_spread, y_top),  // top right
        Point2f(0, y_bottom),                                  // bottom left
        Point2f(imageSize.width, y_bottom)                     // bottom right
    };

    Point2f dst[4] = {
        Point2f(mapSize.width / 2.f - rectangle_w / 2.f, 0),  // top left
        Point2f(mapSize.width / 2.f + rectangle_w / 2.f, 0),  // top right
        Point2f(mapSize.width / 2.f - rectangle_w / 2.f,
                mapSize.height),  // bottom left
        Point2f(mapSize.width / 2.f + rectangle_w / 2.f,
                mapSize.height)  // bottom right
    };

    transform_matrix = getPerspectiveTransform(src, dst);
}

void TransformImage(const sensor_msgs::ImageConstPtr& msg, string& topic) {
    // if no one is listening or the transform is undefined, give up
    if (transform_pubs[topic].getNumSubscribers() == 0 || transform_matrix.empty()) {
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    const Mat& inimage = cv_ptr->image;

    Mat warp_img;
    warpPerspective(inimage, warp_img, transform_matrix, mapSize);

    double map_length = camera_dist_max + cam_mount_x;
    Mat outimage(static_cast<int>(map_length * px_per_meter), warp_img.cols, CV_8UC1, Scalar(0));
    Rect out_warp_roi(0, 0, warp_img.cols, warp_img.rows);

    warp_img.copyTo(outimage(out_warp_roi));

    sensor_msgs::Image outmsg;
    cv_ptr->image = outimage;
    cv_ptr->toImageMsg(outmsg);
    transform_pubs[topic].publish(outmsg);
}

int main(int argc, char** argv) {
    init(argc, argv, "image_transform");
    NodeHandle nh;
    NodeHandle pnh("~");

    bool all_defined = true;
    all_defined &= pnh.getParam("px_per_meter", px_per_meter);
    all_defined &= pnh.getParam("map_dist_max", camera_dist_max);
    all_defined &= pnh.getParam("map_dist_min", camera_dist_min);

    std::string camera_info_topic;
    all_defined &= pnh.getParam("camera_info_topic", camera_info_topic);
    std::string camera_link_name;
    all_defined &= pnh.getParam("camera_link_name", camera_link_name);

    // the launch file can provide camera information in case camera_info is not
    // published
    all_defined &= pnh.getParam("fallback_fov_horizontal", camera_fov_horizontal);
    all_defined &= pnh.getParam("fallback_fov_vertical", camera_fov_vertical);
    all_defined &= pnh.getParam("fallback_image_width", imageSize.width);
    all_defined &= pnh.getParam("fallback_image_height", imageSize.height);

    if (!all_defined) {
        ROS_WARN("[Image Transform] Not all launch params defined");
    }

    // load camera geometry
    rr::CameraGeometry cam_geom;
    cam_geom.LoadInfo(nh, camera_info_topic, camera_link_name, 60.0);

    // set relevant camera geometry fields for this node
    camera_fov_horizontal = cam_geom.GetFOVHorizontal();
    camera_fov_vertical = cam_geom.GetFOVVertical();
    cam_mount_angle = std::get<1>(cam_geom.GetCameraOrientationRPY());
    cam_mount_height = cam_geom.GetCameraLocation().z;
    cam_mount_x = cam_geom.GetCameraLocation().x;

    setTransformFromGeometry();
    ROS_INFO("Calculated perspective transform. Used height %f and angle %f", cam_mount_height, cam_mount_angle);

    string topicsConcat;
    pnh.getParam("transform_topics", topicsConcat);
    vector<string> topics;
    boost::split(topics, topicsConcat, boost::is_any_of(" ,"));
    vector<Subscriber> transform_subs;
    ROS_INFO_STREAM("Found " << topics.size() << " topics in param.");
    for (const string& topic : topics) {
        if (topic.size() == 0) {
            continue;
        }

        transform_subs.push_back(nh.subscribe<sensor_msgs::Image>(topic, 1, boost::bind(TransformImage, _1, topic)));
        ROS_INFO_STREAM("Image_transform subscribed to " << topic);
        string newTopic(topic + "_transformed");
        ROS_INFO_STREAM("Creating new topic " << newTopic);
        transform_pubs[topic] = nh.advertise<sensor_msgs::Image>(newTopic, 1);
    }

    spin();

    return 0;
}
