//
// Created by charlie on 5/25/21.
//

#ifndef RR_EVGP_TEST_CONE_CONNECTION_H
#define RR_EVGP_TEST_CONE_CONNECTION_H

#include <XmlRpcValue.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/publisher.h>

class test_cone_connection {
  private:
    ros::Publisher points;
    typedef std::function<geometry_msgs::PoseArray(geometry_msgs::PoseArray)> shape_func;

    static geometry_msgs::PoseArray parse_points(XmlRpc::XmlRpcValue &);
    void publish_intermittently(geometry_msgs::PoseArray (&get_next_points)(geometry_msgs::PoseArray));

  public:
    test_cone_connection(ros::NodeHandle, const std::string &, XmlRpc::XmlRpcValue);

    static geometry_msgs::PoseArray line(geometry_msgs::PoseArray previous);
};

#endif  // RR_EVGP_TEST_CONE_CONNECTION_H
