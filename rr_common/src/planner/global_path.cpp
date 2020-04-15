/**
 * Used to determine the cost for a plan (list of poses) based on the distance from a global
 * path in the world. It takes a nav_msgs::Path and compares the plan to the global path.
 * @Note: Assumes plan is "close" to path and we are progressing forward along global path
 *
 * @author Brian Cochran
 */

#include <rr_common/planning/global_path.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <parameter_assertions/assertions.h>

namespace rr {

GlobalPath::GlobalPath(ros::NodeHandle nh) : listener_(new tf::TransformListener) {
    std::string global_path_topic;
    assertions::getParam(nh, "global_path_topic", global_path_topic);
    assertions::getParam(nh, "robot_base_frame", robot_base_frame_);

    assertions::getParam(nh, "cost_scaling_factor", cost_scaling_factor_, { assertions::greater_eq(0.0) });

    global_path_sub_ = nh.subscribe(global_path_topic, 1, &GlobalPath::SetPathMessage, this);
}

std::vector<double> GlobalPath::CalculateCost(const std::vector<PathPoint>& plan) {
    std::vector<double> costs(plan.size());
    std::transform(plan.cbegin(), plan.cend(), costs.begin(),
                   [this](const PathPoint& p) { return CalculateCost(p.pose); });
    return costs;
}

double GlobalPath::CalculateCost(const Pose& planPose) {
    //iterate through global path until distance increases
    unsigned int currIndex = 0;//last_used_point_index_; //#TODO
    unsigned int nextIndex = this->GetNextIndex(currIndex);

    tf::Pose w_planPose = robot_to_path_transform_ * tf::Pose(tf::createQuaternionFromYaw(0), tf::Vector3(planPose.x, planPose.y, 0));

    while (nextIndex != currIndex) { //full loop
        tf::Pose w_currPathPose;
        tf::poseMsgToTF(global_path_msg_.poses[currIndex].pose, w_currPathPose);
        double distCurr = GlobalPath::GetPointDistance(w_planPose, w_currPathPose);

        tf::Pose w_nextPathPose;
        tf::poseMsgToTF(global_path_msg_.poses[nextIndex].pose, w_nextPathPose);
        double distNext = GlobalPath::GetPointDistance(w_planPose, w_nextPathPose);

        if (distNext > distCurr) {
            last_used_point_index_ = currIndex;
            return distCurr;
        }
        currIndex = nextIndex;
        nextIndex = this->GetNextIndex(nextIndex);
    }
    return 0;
}

void GlobalPath::PreProcess() {
    this->LookupPathTransform();
    //closest_point_to_robot_index = //#TODO

}

void GlobalPath::LookupPathTransform() {
    accepting_updates_ = false;
    try {
        listener_->waitForTransform(global_path_msg_.header.frame_id, robot_base_frame_, ros::Time(0), ros::Duration(.05));
        listener_->lookupTransform(global_path_msg_.header.frame_id, robot_base_frame_, ros::Time(0), robot_to_path_transform_);
    } catch (tf::TransformException& ex) {
        ROS_ERROR_STREAM(ex.what());
    }
    accepting_updates_ = true;
}

double GlobalPath::GetPointDistance(tf::Pose pose1, tf::Pose pose2) {
    return std::abs(pose1.getOrigin().distance(pose2.getOrigin()));
}

unsigned int GlobalPath::GetNextIndex(unsigned int i) {
    unsigned int numElements = sizeof(global_path_msg_.poses)/sizeof(global_path_msg_.poses[0]);
    if (i + 1 >= numElements) {
        return 0; //wrap around
    } else {
        return i + 1;
    }
}

void GlobalPath::SetPathMessage(const nav_msgs::Path& path_msg) {
    if (!accepting_updates_) {
        return;
    }
    last_used_point_index_ = 0;
    global_path_msg_ = nav_msgs::Path(path_msg);

    updated_ = true;
}

}  // namespace rr
