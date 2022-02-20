#ifndef PIDNODE_HPP_
#define PIDNODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rr_msgs/msg/pid.hpp>

namespace rr_rviz_plugins
{
    class PidNode : public rviz_common::RosTopicDisplay<rr_msgs::msg::Pid>
    {

        public:
            PidNode(rclcpp::Node & node);
            override void processMessage(rr_msgs::msg::Pid);
    };
}

#endif