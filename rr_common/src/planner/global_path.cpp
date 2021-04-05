/**
 * Used to determine the cost for a plan (list of poses) based on the distance from a global
 * path in the world. It takes a nav_msgs::Path and compares the plan to the global path.
 * @Note: Assumes plan is "close" to path and we are progressing forward along global path
 *
 * @authors Brian Cochran, Nico Bartholomai
 */

#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <parameter_assertions/assertions.h>
#include <rr_common/planning/global_path.h>

#include <cmath>
#include <numeric>
#include <tuple>

namespace rr {

GlobalPath::GlobalPath(ros::NodeHandle nh) : has_global_path_(false), listener_(new tf::TransformListener) {
    std::string global_path_topic;
    nh.param<std::string>("global_path_topic", global_path_topic, "/global_center_path");
    nh.param<std::string>("robot_base_frame", robot_base_frame_, "base_footprint");
    nh.param<double>("dtw_window_factor", dtw_window_factor_, 0.25);
    nh.param<double>("progress_seg_factor", progress_seg_factor_, 1.1);

    global_path_sub_ = nh.subscribe(global_path_topic, 1, &GlobalPath::SetPathMessage, this);
    global_path_seg_pub_ = nh.advertise<nav_msgs::Path>("/global_path_seg", 1);
    progress_seg_pub_ = nh.advertise<nav_msgs::Path>("/progress_path_seg", 1);
}

double GlobalPath::CalculateCost(const std::vector<PathPoint> &plan) {
    if (!has_global_path_) {
        return 0.0;
    }
    // convert points to world frame
    std::vector<tf::Point> sample_path = convertToWorldPoints(plan);

    // get the global segment
    std::vector<tf::Point> global_segment = GlobalPath::getGlobalSegment(sample_path);

    // use dtw to comp sample to global
    int window = (int)(dtw_window_factor_ * std::max(global_segment.size(), sample_path.size()));
    double dtw_val = GlobalPath::dtwDistance(global_segment, sample_path, window);
    return dtw_val;
}

void GlobalPath::vizDTWSegment(const std::vector<PathPoint> &plan) {
    if (!has_global_path_) {
        return;
    }

    std::vector<tf::Point> sample_path = convertToWorldPoints(plan);
    std::vector<tf::Point> global_segment = getGlobalSegment(sample_path);

    nav_msgs::Path global_seg_msg;  // convert type
    for (const tf::Point &path_point : global_segment) {
        geometry_msgs::PoseStamped ps;
        ps.pose.position.x = path_point.getX();
        ps.pose.position.y = path_point.getY();
        global_seg_msg.poses.push_back(ps);
    }
    global_seg_msg.header.frame_id = global_path_frame_;
    global_path_seg_pub_.publish(global_seg_msg);
}

void GlobalPath::vizProgressSegment(const std::vector<PathPoint> &plan) {
    if (!has_global_path_) {
        return;
    }

    // convert points to world frame
    std::vector<tf::Point> sample_path = convertToWorldPoints(plan);

    // Get the index of the closest global points to the beginning and end of the sample plan
    int seg_start_index = closestPt(global_path_, sample_path[0]);
    std::vector<double> path_adj_dist = adjacentDistances(sample_path);
    double path_len = std::accumulate(path_adj_dist.begin(), path_adj_dist.end(), 0.0);
    path_len *= progress_seg_factor_;  // add some buffer room for the segment
    int seg_end_index = ptAfterDist(global_cum_dist_, seg_start_index, path_len);

    // Get the length of the global path segment this creates.
    std::vector<tf::Point> global_segment = getPathSegment(global_path_, seg_start_index, seg_end_index);

    nav_msgs::Path global_seg_msg;  // convert type
    for (const auto path_point : global_segment) {
        geometry_msgs::PoseStamped ps;
        ps.pose.position.x = path_point.getX();
        ps.pose.position.y = path_point.getY();
        global_seg_msg.poses.push_back(ps);
    }
    global_seg_msg.header.frame_id = "map";
    progress_seg_pub_.publish(global_seg_msg);
}

std::vector<tf::Point> GlobalPath::convertToWorldPoints(const std::vector<PathPoint> &plan) {
    std::vector<tf::Point> tf_path(plan.size());
    std::transform(plan.begin(), plan.end(), tf_path.begin(), [&](const PathPoint &pose) {
        tf::Pose w_pose = robot_to_path_transform_ * tf::Pose(tf::createQuaternionFromYaw(pose.pose.theta),
                                                              tf::Vector3(pose.pose.x, pose.pose.y, 0));
        return tf::Point(w_pose.getOrigin().x(), w_pose.getOrigin().y(), 0);
    });
    return tf_path;
}

int GlobalPath::closestPt(const std::vector<tf::Point> &path, const tf::Point &pt) {
    std::vector<double> distances_to_pt(path.size());
    std::transform(path.begin(), path.end(), distances_to_pt.begin(),
                   [pt](const tf::Point &path_pt) { return GlobalPath::GetPointDistance(path_pt, pt); });
    return std::min_element(distances_to_pt.begin(), distances_to_pt.end()) - distances_to_pt.begin();
}

std::vector<tf::Point> GlobalPath::getPathSegment(const std::vector<tf::Point> &path, int start, int end) {
    std::vector<tf::Point> segment;
    if (start <= end) {  // assume sample path length < global path length
        segment = std::vector<tf::Point>(path.begin() + start, path.begin() + end);
    } else {
        segment = std::vector<tf::Point>(path.begin() + start, path.end());
        segment.insert(segment.end(), path.begin(), path.begin() + end);
    }
    return segment;
}

int GlobalPath::ptAfterDist(const std::vector<double> &cum_dist, int start, double len) {
    // account for the distance going beyond the end of the path
    double upper_cum_limit = std::fmod((cum_dist[start] + len), cum_dist.back());
    // get the index of the closest point that lies after the len
    int index = std::upper_bound(cum_dist.begin(), cum_dist.end(), upper_cum_limit) - global_cum_dist_.begin();
    return index;
}

double GlobalPath::GetLocalPathProgress(const std::vector<PathPoint> &plan, const bool viz) {
    if (!has_global_path_) {
        return 0.0;
    }
    // convert points to world frame
    std::vector<tf::Point> sample_path = convertToWorldPoints(plan);

    // Get the index of the closest global points to the beginning and end of the sample plan
    int seg_start_index = closestPt(global_path_, sample_path[0]);
    std::vector<double> path_adj_dist = adjacentDistances(sample_path);
    double path_len = std::accumulate(path_adj_dist.begin(), path_adj_dist.end(), 0.0);
    path_len *= progress_seg_factor_;  // add some buffer for the global seg len
    int seg_end_index = ptAfterDist(global_cum_dist_, seg_start_index, path_len);

    // Get the length of the global path segment this creates.
    std::vector<tf::Point> global_segment = getPathSegment(global_path_, seg_start_index, seg_end_index);
    std::vector<double> global_adj_dist = GlobalPath::adjacentDistances(global_segment);
    double plan_len = std::accumulate(global_adj_dist.begin(), global_adj_dist.end(), 0.0);

    // normalize and invert so shorter paths give higher costs
    double global_path_percent = plan_len / global_path_len_;  // assumes all paths are shorter than the global path

    return 1.0 - global_path_percent;
}

std::vector<tf::Point> GlobalPath::getGlobalSegment(const std::vector<tf::Point> &sample_path) {
    // get the end points
    int seg_start_index = closestPt(global_path_, sample_path[0]);
    std::vector<double> sample_adj_dist = adjacentDistances(sample_path);
    double sample_length = std::accumulate(sample_adj_dist.begin(), sample_adj_dist.end(), 0.0);
    int seg_end_index = ptAfterDist(global_cum_dist_, seg_start_index, sample_length);

    // return the global seg that lies btwn the endpts
    return GlobalPath::getPathSegment(global_path_, seg_start_index, seg_end_index);
}

// follows the pseudocode found in https://en.wikipedia.org/wiki/Dynamic_time_warping
double GlobalPath::dtwDistance(const std::vector<tf::Point> &path1, const std::vector<tf::Point> &path2, int w) {
    int n = path1.size();
    int m = path2.size();

    std::vector<std::vector<double>> dtw(n + 1, std::vector<double>(m + 1, std::numeric_limits<double>::infinity()));
    w = std::max(w, std::abs(n - m));  // adapt window size
    dtw[0][0] = 0;

    for (int i = 1; i < n + 1; i++) {
        for (int j = std::max(1, i - w); j < std::min(m, i + w) + 1; j++) {
            double cost = tf::tfDistance(path1[i - 1], path2[j - 1]);
            dtw[i][j] = cost + std::min({ dtw[i - 1][j], dtw[i][j - 1], dtw[i - 1][j - 1] });
        }
    }

    return dtw[n][m];
}

std::vector<double> GlobalPath::adjacentDistances(const std::vector<tf::Point> &path) {
    std::vector<double> distances(1, 0.0);
    std::transform(path.begin(), path.end() - 1, path.begin() + 1, std::back_inserter(distances),
                   &GlobalPath::GetPointDistance);
    return distances;
}

void GlobalPath::PreProcess() {
    if (!has_global_path_) {
        return;
    }
    try {
        listener_->waitForTransform(global_path_frame_, robot_base_frame_, ros::Time(0), ros::Duration(.05));
        listener_->lookupTransform(global_path_frame_, robot_base_frame_, ros::Time(0), robot_to_path_transform_);
    } catch (tf::TransformException &ex) {
        ROS_ERROR_STREAM(ex.what());
    }
}

double GlobalPath::GetPointDistance(const tf::Point &point1, const tf::Point &point2) {
    return point1.distance(point2);
}

void GlobalPath::SetPathMessage(const nav_msgs::Path &global_path_msg) {
    has_global_path_ = true;
    global_path_frame_ = global_path_msg.header.frame_id;

    // Convert Type
    global_path_ = std::vector<tf::Point>(global_path_msg.poses.size());
    std::transform(global_path_msg.poses.begin(), global_path_msg.poses.end(), global_path_.begin(),
                   [](const auto &poseStamped) {
                       tf::Pose tf_pose;
                       tf::poseMsgToTF(poseStamped.pose, tf_pose);
                       return tf_pose.getOrigin();
                   });

    // Get Distances
    std::vector<double> adj_dist = GlobalPath::adjacentDistances(global_path_);
    global_cum_dist_ = std::vector<double>(adj_dist.size());
    std::partial_sum(adj_dist.begin(), adj_dist.end(), global_cum_dist_.begin());
    global_path_len_ = std::accumulate(global_cum_dist_.begin(), global_cum_dist_.end(), 0.0);
}

}  // namespace rr
