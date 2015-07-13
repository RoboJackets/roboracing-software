#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

#include <iarrc/constants.hpp>
#include <cmath>
#include <iarrc_msgs/iarrc_steering.h>


using namespace cv_bridge;
using namespace sensor_msgs;

ros::Publisher world_pub;

double delta_theta = M_PI / constants::radial_steps;
double delta_r = 1;


cv::Point PolarToCartsesian(double r, double theta) {
  return cv::Point(r * cos(theta) + constants::origin_y, constants::origin_x - r * sin(theta));
}

int toDegrees(double t) {
    return t * 180/M_PI;
}

void plannerCB(const sensor_msgs::Image::ConstPtr& msg) {
    boost::shared_ptr<CvImage> worldPtr = toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    double max_t = 0;
    int max_accum = -1010101010;
    for (double t = 0; t < M_PI; t += delta_theta) {
        int accum = 0;
        for (int r = 0; r < constants::image_size / 2; r+= delta_r) {
            auto p = PolarToCartsesian(r,t);
            accum += worldPtr.get()->image.at<uchar>(p.y, p.x);
        }
        if (accum > max_accum) {
            max_t = t;
        }
    }

    iarrc_msgs::iarrc_steering pmsg;
    pmsg.angle = toDegrees(max_t) + 90;
    world_pub.publish(pmsg);
}



int main(int argc, char* argv[]) {
    ros::init(argc, argv, "planner");
    std::string world_model_topic;
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    nhp.param(std::string("world_model_topic"), world_model_topic, std::string("/world_model"));

    ros::Subscriber obstacle_sub = nh.subscribe(world_model_topic, 1, plannerCB);

    world_pub = nh.advertise<iarrc_msgs::iarrc_steering>("/plan", 1);
    ros::spin();
}
