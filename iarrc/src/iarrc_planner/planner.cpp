#define DEBUG false //make me true to see pretty pictures

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

double delta_theta = (M_PI / 3) / constants::radial_steps;
double delta_r = 1;

int dilationSize = (constants::wheel_base * constants::pixels_per_meter) / 2;
cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*dilationSize + 1, 2*dilationSize + 1), cv::Point(dilationSize, dilationSize));
    

cv::Point polarToCartesian(double r, double theta) {
  return cv::Point(r * cos(theta) + constants::origin_y, (constants::origin_x) - r * sin(theta));
}

int toDegrees(double t) {
    return t * 180/M_PI;
}

void plannerCB(const sensor_msgs::Image::ConstPtr& msg) {
    boost::shared_ptr<CvImage> worldPtr = toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat workingImage;
    cv::erode(worldPtr.get()->image, workingImage, element);
    
    double max_t = 0;
    int max_accum = 0;
    for (double t = M_PI / 3; t < 2*M_PI/3; t += delta_theta) {
        int accum = 0;
        for (int r = 0; r < constants::image_size - 50; r+= delta_r) {
	  cv::Point p = polarToCartesian(r, t);
	  if (workingImage.at<uchar>(p) < (uchar) 255) {
	    break;
	  }
	  accum += 100 - toDegrees(std::abs(t - M_PI_2));
        }
        if (accum > max_accum) {
	  max_t = t;
	  max_accum = accum;
	} else if (max_accum == accum) {
	  if (abs(t - M_PI_2) < abs(max_t - M_PI_2)) {
	    max_t = t;
	  }
	}
    }

    if (DEBUG) {
      cv::line(workingImage, polarToCartesian(0, 0), polarToCartesian(750, max_t), 0, 10);
      cv::namedWindow("Debug");
      cv::imshow("Debug", workingImage);
      cv::waitKey(100);
    }

    iarrc_msgs::iarrc_steering pmsg;
    pmsg.angle = toDegrees(max_t) - 90;
    world_pub.publish(pmsg);
}



int main(int argc, char* argv[]) {
    ros::init(argc, argv, "planner");
    std::string world_model_topic;
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    nhp.param(std::string("world_model_topic"), world_model_topic, std::string("/world_model"));

    ros::Subscriber obstacle_sub = nh.subscribe(world_model_topic, 1, plannerCB);

    world_pub = nh.advertise<iarrc_msgs::iarrc_steering>("/steering", 1);
    ros::spin();
}
