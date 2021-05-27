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
    bool _execute;

    static geometry_msgs::PoseArray parse_points(XmlRpc::XmlRpcValue &);
    void start(const std::function<void(void)> &func);

  public:
    test_cone_connection(ros::NodeHandle, const std::string &, XmlRpc::XmlRpcValue);
};

#endif  // RR_EVGP_TEST_CONE_CONNECTION_H
