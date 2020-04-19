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
#include <tuple>
#include <parameter_assertions/assertions.h>

namespace rr {

GlobalPath::GlobalPath(ros::NodeHandle nh) : has_global_path_(false), accepting_updates_(true), closest_point_to_robot_index_(0), listener_(new tf::TransformListener) {
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
    if (!has_global_path_) {
        return 0.0;
    }
    tf::Pose w_planPose = robot_to_path_transform_ * tf::Pose(tf::createQuaternionFromYaw(0), tf::Vector3(planPose.x, planPose.y, 0));

    return std::get<1>(this->FindNearestPathPointIndex(closest_point_to_robot_index_, w_planPose));
}

void GlobalPath::PreProcess() {
    if (!has_global_path_) {
        return;
    }
    this->LookupPathTransform();
    //find nearest point to robot
    tf::Pose w_robotPose = robot_to_path_transform_ * tf::Pose(tf::createQuaternionFromYaw(0), tf::Vector3(0, 0, 0));

    std::tuple<unsigned int, double> closestIndexPoint = this->FindNearestPathPointIndex(0, w_robotPose);
    closest_point_to_robot_index_ = std::get<0>(closestIndexPoint);
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

std::tuple<unsigned int, double> GlobalPath::FindNearestPathPointIndex(unsigned int startIndex, tf::Pose inputPose) {
    if (!has_global_path_) {
        return std::make_tuple(0, 0.0);
    }

    unsigned int currIndex = startIndex;
    unsigned int nextIndex = (currIndex + 1 ) % global_path_msg_.poses.size();

    tf::Pose w_currPathPose;
    tf::poseMsgToTF(global_path_msg_.poses[currIndex].pose, w_currPathPose);
    double distCurr = GlobalPath::GetPointDistance(inputPose, w_currPathPose);

    tf::Pose w_nextPathPose;
    tf::poseMsgToTF(global_path_msg_.poses[nextIndex].pose, w_nextPathPose);
    double distNext = GlobalPath::GetPointDistance(inputPose, w_nextPathPose);

    do {
        if (distNext >= distCurr) {
            return std::make_tuple(currIndex, distCurr);
        }

        currIndex = nextIndex;
        nextIndex = (currIndex + 1 ) % global_path_msg_.poses.size();

        w_currPathPose = w_nextPathPose;
        distCurr = distNext;

        tf::poseMsgToTF(global_path_msg_.poses[nextIndex].pose, w_nextPathPose);
        distNext = GlobalPath::GetPointDistance(inputPose, w_nextPathPose);
    } while (currIndex != startIndex); //full loop around

    return std::make_tuple(0, 0.0); //nothing found
}

void GlobalPath::SetPathMessage(const nav_msgs::Path& path_msg) {
    if (!accepting_updates_) {
        return;
    }
    has_global_path_ = true;
    global_path_msg_ = nav_msgs::Path(path_msg);

    updated_ = true;
}

}  // namespace rr
