//
// Created by charlie on 3/29/21.
//

#include "lidar_cone_parsing.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_cone_parsing");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    lidar_cone_parsing parsing = lidar_cone_parsing(&nh);

    ros::spin();
    return 0;
}

lidar_cone_parsing::lidar_cone_parsing(ros::NodeHandle *nh) {
    camera_subscriber = nh->subscribe("/camera/image_color_rect", 5, &lidar_cone_parsing::camera_callback, this);
}

void lidar_cone_parsing::camera_callback(const sensor_msgs::Image::ConstPtr &msg) {

}
