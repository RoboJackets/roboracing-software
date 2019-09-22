#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>

ros::Publisher steering_pub;
ros::Publisher speed_pub;

using namespace cv;
using namespace std;

vector<Rect> regions;

double min_steering;
double steering_rate;

double center_discount;

void image_callback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("CV-Bridge error: %s", e.what());
        return;
    }

    Mat &frame = cv_ptr->image;
    Mat blue_mask;
    inRange(frame, Scalar(255, 0, 0), Scalar(255, 0, 0), blue_mask);

    blue_mask = ~blue_mask;

    // frame *= blue_mask;

    Mat frame_1channel;

    cvtColor(frame, frame_1channel, CV_BGR2GRAY);

    vector<int> counts;
    counts.reserve(regions.size());

    for (const auto &region : regions) {
        counts.push_back(countNonZero(frame_1channel(region)));
    }

    // Discount center region to encourage forward motion
    counts[(counts.size() / 2)] *= center_discount;

    auto min_iter = min_element(counts.begin(), counts.end());

    auto index = distance(counts.begin(), min_iter);

    auto steering_val = min_steering + (steering_rate * index);

    rr_platform::steering steering_msg;
    steering_msg.header.stamp = ros::Time::now();
    steering_msg.angle = -steering_val;
    steering_pub.publish(steering_msg);

    rr_platform::speed speed_msg;
    speed_msg.header.stamp = ros::Time::now();
    if (steering_msg.angle == 0) {
        speed_msg.speed = 1.5;
    } else {
        speed_msg.speed = 1.0;
    }
    speed_pub.publish(speed_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ol_reliable");

    ros::NodeHandle nh;
    ros::NodeHandle pnh{ "~" };

    pnh.param("center_discount", center_discount, 0.85);

    int num_regions;
    pnh.param("num_regions", num_regions, 3);

    int regionWidth = 160 / num_regions;
    for (int i = 0; i < num_regions; i++) {
        regions.push_back(Rect(i * regionWidth, 0, regionWidth, 90));
    }

    pnh.param("min_steering", min_steering, -0.35);

    double max_steering;
    pnh.param("max_steering", max_steering, 0.35);

    steering_rate = (max_steering - min_steering) / (num_regions - 1);

    steering_pub = nh.advertise<rr_platform::steering>("/steering", 1);
    speed_pub = nh.advertise<rr_platform::speed>("/speed", 1);

    image_transport::ImageTransport imageTransport{ nh };

    auto img_sub = imageTransport.subscribe("/colors_img", 1, image_callback);

    ros::spin();

    return 0;
}
