#include "ros/ros.h"
#include "tag_detection.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "april_tag_pointcloud");
    ros::NodeHandle nhp("~");
    ros::NodeHandle nh;
    std::string camera_frame, pointcloud, tag_detections_topic, tag_detection_markers, destination_frame;
    double x_offset, y_offset, px_per_m, width, height;
    XmlRpc::XmlRpcValue tags;
    nhp.getParam("camera_frame", camera_frame);
    nhp.getParam("pointcloud", pointcloud);
    nhp.getParam("tag_detections_topic", tag_detections_topic);
    nhp.getParam("tag_detection_markers", tag_detection_markers);
    nhp.getParam("destination_frame", destination_frame);
    nhp.getParam("x_offset", x_offset);
    nhp.getParam("y_offset", y_offset);
    nhp.getParam("px_per_m", px_per_m);
    nhp.getParam("width", width);
    nhp.getParam("height", height);
    nhp.getParam("tags", tags);
    tag_detection tagDetection(&nh, camera_frame, pointcloud, tag_detections_topic, destination_frame,
                               tag_detection_markers, x_offset,
                               y_offset, px_per_m, width, height);

    ros::spin();
    return 0;
}