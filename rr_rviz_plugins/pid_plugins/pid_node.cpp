#include "pid_node.hpp"

#include <pluginlib/class_list_macros.hpp>

void PidNode::processMessage(rr_msgs::msg::Pid) {}

PLUGINLIB_EXPORT_CLASS(myplugin::Pid, rviz_common::Display)