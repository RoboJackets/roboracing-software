/**
 * Used to determine the cost for a plan (list of poses) based on the distance from a global
 * path in the world. It takes a nav_msgs::Path and compares the plan to the global path.
 * @Note: Assumes plan is "close" to path and we are progressing forward along global path
 *
 * @authors Brian Cochran, Nico Bartholomai
 */

#include <rr_common/planning/global_path.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <tuple>
#include <parameter_assertions/assertions.h>
#include <numeric>
#include <rr_common/planning/vectordtw.h>

namespace rr {

GlobalPath::GlobalPath(ros::NodeHandle nh) : has_global_path_(false), accepting_updates_(true), listener_(new tf::TransformListener) {
    std::string global_path_topic;
    assertions::getParam(nh, "global_path_topic", global_path_topic);
    assertions::getParam(nh, "robot_base_frame", robot_base_frame_);
    assertions::getParam(nh, "cost_scaling_factor", cost_scaling_factor_, { assertions::greater_eq(0.0) });

    global_path_sub_ = nh.subscribe(global_path_topic, 1, &GlobalPath::SetPathMessage, this);
    global_path_seg_pub_ = nh.advertise<nav_msgs::Path>("/global_path_seg", 1);
}

double GlobalPath::CalculateCost(const std::vector<PathPoint> &plan, const bool viz) {
    if (!has_global_path_) {
        return 0.0;
    }
    std::vector<tf::Point> sample_path(plan.size());
    //convert to world frame
    this->LookupPathTransform();
    std::transform(plan.begin(), plan.end(), sample_path.begin(), [&](const PathPoint &pose) {
        tf::Pose w_pose = robot_to_path_transform_ * tf::Pose(tf::createQuaternionFromYaw(pose.pose.theta),
                                                              tf::Vector3(pose.pose.x, pose.pose.y, 0));
        return tf::Point(w_pose.getOrigin().x(), w_pose.getOrigin().y(), 0);
    });
    //get global segment
    tf::Point sample_start = sample_path[0];
    std::vector<tf::Point> sample_path_ft(plan.size());
    std::vector<double> distances_to_origin(global_path_.size());
    std::transform(global_path_.begin(), global_path_.end(), distances_to_origin.begin(),
                   [sample_start](const tf::Point &global_pnt) {
                       return GlobalPath::GetPointDistance(global_pnt, sample_start);
                   });
    int seg_start_index = std::min_element(distances_to_origin.begin(), distances_to_origin.end())
            - distances_to_origin.begin();
    vector<double> sample_adj_dist = GlobalPath::adjacent_distances(sample_path);
    double sample_length = std::accumulate(sample_adj_dist.begin(), sample_adj_dist.end(), 0.0);
    double upper_cum_limit = std::fmod((global_cum_dist_[seg_start_index] + sample_length),
                                       global_cum_dist_[global_cum_dist_.size() - 1]);
    int seg_end_index = std::upper_bound(global_cum_dist_.begin(), global_cum_dist_.end(), upper_cum_limit)
            - global_cum_dist_.begin();
    std::vector<tf::Point> global_segment;
    if (seg_start_index <= seg_end_index) {    //Assume sample_path.length < global_path.length
        global_segment = std::vector<tf::Point>(global_path_.begin() + seg_start_index,
                                                global_path_.begin() + seg_end_index);
    } else {
        global_segment = std::vector<tf::Point>(global_path_.begin() + seg_start_index, global_path_.end());
        global_segment.insert(global_segment.end(), global_path_.begin(), global_path_.begin() + seg_end_index);
    }
    //dtw
    std::vector<Point> global_points(global_segment.size());
    std::transform(global_segment.begin(), global_segment.end(), global_points.begin(), [](const tf::Point &tf_pt) {
        return Point(tf_pt.getX(), tf_pt.getY(), 0);
    });
    std::vector<Point> sample_points(sample_path.size());
    std::transform(sample_path.begin(), sample_path.end(), sample_points.begin(), [](const tf::Point &tf_pt) {
        return Point(tf_pt.getX(), tf_pt.getY(), 0);
    });
    int points_len = std::min(global_points.size(), sample_points.size());
    //vectors must be same length for fast dtw, so similar point density assumed
    if (global_points.size() != sample_points.size()) {
        global_points.resize(points_len);
        sample_points.resize(points_len);
    }
    if (viz) {
        //convert type for publishing
        nav_msgs::Path global_seg_msg;
        for (auto path_point : global_points) {
            geometry_msgs::PoseStamped ps;
            ps.pose.position.x = path_point.x;
            ps.pose.position.y = path_point.y;
            global_seg_msg.poses.push_back(ps);
        }
        global_seg_msg.header.frame_id = "map";
        global_path_seg_pub_.publish(global_seg_msg);
    }
    VectorDTW dtw1(points_len, points_len / 10);
    double dtw_val = dtw1.fastdynamic(global_points, sample_points);
    return dtw_val;
}

std::vector<double> GlobalPath::adjacent_distances(const std::vector<tf::Point>& path) {
    std::vector<double> distances(1, 0.0);
    std::transform(path.begin(), path.end() - 1, path.begin() + 1, std::back_inserter(distances), &GlobalPath::GetPointDistance);
    return distances;
}

void GlobalPath::PreProcess() {
    if (!has_global_path_) {
        return;
    }
    this->LookupPathTransform();
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

double GlobalPath::GetPointDistance(tf::Point point1, tf::Point point2){
    return point1.distance(point2);
}

void GlobalPath::SetPathMessage(const nav_msgs::Path& global_path_msg) {
    if (!accepting_updates_) {
        return;
    }
    has_global_path_ = true;
    global_path_msg_ = nav_msgs::Path(global_path_msg);
    global_path_ = std::vector<tf::Point> (global_path_msg.poses.size());
    // Convert Type
    std::transform(global_path_msg.poses.begin(), global_path_msg.poses.end(), global_path_.begin(),
                   [](const auto &poseStamped) {
        tf::Pose tf_pose;
        tf::poseMsgToTF(poseStamped.pose, tf_pose);
        return tf_pose.getOrigin();
    });
    // Get Distances
    std::vector<double> adj_dist = GlobalPath::adjacent_distances(global_path_);
    global_cum_dist_ = std::vector<double>(adj_dist.size());
    std::partial_sum(adj_dist.begin(), adj_dist.end(), global_cum_dist_.begin());
    updated_ = true;
}

}  // namespace rr
