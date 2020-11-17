#include "tag_detection.h"

void callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
    std::uint32_t second = msg->header.stamp.sec;
    auto msgs = msg->detections;
    for (const auto& message : msgs) {
        ROS_INFO_STREAM((message.id[0]));
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tag_detection");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("tag_detections", 1, callback);
    ros::spin();
    return 0;
}
