//
// Created by Charlie Jenkins on 3/29/21.
//

#ifndef ROBORACING_SOFTWARE_LIDAR_CONE_PARSING_H
#define ROBORACING_SOFTWARE_LIDAR_CONE_PARSING_H

#include "ros/ros.h"
#include "sensor_msgs/Image.h"

class lidar_cone_parsing {
  public:
    explicit lidar_cone_parsing(ros::NodeHandle *);
  private:
    ros::Subscriber camera_subscriber;
    void camera_callback(const sensor_msgs::Image::ConstPtr&);
};

#endif  // ROBORACING_SOFTWARE_LIDAR_CONE_PARSING_H
