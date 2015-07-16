#include <stdio.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <iarrc/constants.hpp>


using namespace cv;
using namespace std;
using namespace ros;
using namespace pcl;
using namespace constants;

Publisher obstacle_img_pub;
sensor_msgs::Image obstacle_img;
/**
 * calculates the normal probability distribution function for a given set of
 * values
 * @param  x the value to be calculated on
 * @param  m mean
 * @param  s stdvev
 * @return   the density at that distance
 */
double normal_pdf(double x, double m, double s)
{
    //currently unused
    static const double inv_sqrt_2pi = 0.3989422804014327;
    double a = (x - m) / s;
    return inv_sqrt_2pi / s * exp(-double(0.5) * a * a);
}


void convertToImgCB(const sensor_msgs::PointCloud2 cloud_in) {
    Mat img = Mat::ones(world_height, world_width, CV_8U) * 255;
    PointCloud<PointXYZ> cloud;
    fromROSMsg(cloud_in, cloud);
    for (int i=0; i < cloud.size(); i++) {
        float dist = sqrt(pow(cloud[i].y, 2) + pow(cloud[i].x, 2)) * pixels_per_meter;
        circle(img, Point(((cloud[i].y * pixels_per_meter) + origin_y), world_height - ((cloud[i].x * pixels_per_meter) + origin_x)), dist * laser_step_size, 0, -1);
    }

    circle(img, Point(origin_y, origin_x), 0.3 * constants::pixels_per_meter, 255, -1);
    obstacle_img_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg());
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "pointcloud_to_image");
    std::string cloud_topic;
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    nhp.param(std::string("cloud_topic"), cloud_topic, std::string("/cloud"));
    ros::Subscriber cone_detect_sub = nh.subscribe(cloud_topic, 1, convertToImgCB);
    obstacle_img_pub = nh.advertise<sensor_msgs::Image>("/obstacle_img", 1);
    ros::spin();
}
