#include "tag_detection.h"

class tag_detection {
    void callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
        std::uint32_t second = msg->header.stamp.sec;
        ROS_INFO("%i", second);
    }

    int main(int argc, char** argv) {
        ros::init(argc, argv, "apriltag_location");
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("tag_detections", 1, &tag_detection::callback, this);
        ros::spin();
        return 0;
    }
};
