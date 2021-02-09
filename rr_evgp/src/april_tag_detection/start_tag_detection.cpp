#include "ros/ros.h"
#include "tag_detection.h"
#include "start_tag_detection.h"

std::vector<april_robot> constructRobots(XmlRpc::XmlRpcValue value);

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
    std::vector<april_robot> robots = constructRobots(tags);
    tag_detection tagDetection(&nh, camera_frame, pointcloud, tag_detections_topic, destination_frame,
                               tag_detection_markers, x_offset,
                               y_offset, px_per_m, width, height, robots);

    ros::spin();
    return 0;
}

std::vector<april_robot> constructRobots(XmlRpc::XmlRpcValue xml) {
    ROS_ASSERT(xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
    std::vector<april_robot> robots;
    for (int32_t i = 0; i < xml.size(); i++) {
        april_robot robot{};
        XmlRpc::XmlRpcValue xmlRobot = xml[i];
        std::string name = static_cast<std::string>(xmlRobot["name"]);
        robot.name = name;
        ROS_ASSERT(xmlRobot["layout"].getType() == XmlRpc::XmlRpcValue::TypeArray);
        XmlRpc::XmlRpcValue layout = xmlRobot["layout"];

        for (int32_t j = 0; j < layout.size(); j++) {
            XmlRpc::XmlRpcValue xmlTag = layout[j];
            april_tag tag{};
            tag.id = static_cast<int>(xmlTag["id"]);
            tag.size = static_cast<double>(xmlTag["size"]);
            auto x = static_cast<double>(xmlTag["x"]);
            auto y = static_cast<double>(xmlTag["y"]);
            auto z = static_cast<double>(xmlTag["z"]);
            auto qw = static_cast<double>(xmlTag["qw"]);
            auto qx = static_cast<double>(xmlTag["qx"]);
            auto qy = static_cast<double>(xmlTag["qy"]);
            auto qz = static_cast<double>(xmlTag["qz"]);

            tag.pose = tf::Pose(tf::Quaternion(qw, qx, qy, qz), tf::Vector3(x, y, z));
            robot.tags[tag.id] = tag;
        }

        robots.push_back(robot);
    }
    return robots;
}
