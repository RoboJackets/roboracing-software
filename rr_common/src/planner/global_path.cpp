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
    assertions::getParam(nh, "global_path_topic", global_path_topic);
    assertions::getParam(nh, "robot_base_frame", robot_base_frame_);
    assertions::getParam(nh, "dtw_window_factor", dtw_window_factor_);

    global_path_sub_ = nh.subscribe(global_path_topic, 1, &GlobalPath::SetPathMessage, this);
    global_path_seg_pub_ = nh.advertise<nav_msgs::Path>("/global_path_seg", 1);
}

double GlobalPath::CalculateCost(const std::vector<PathPoint> &plan, const bool viz) {
    if (!has_global_path_) {
        return 0.0;
    }
    // convert points to world frame
    std::vector<tf::Point> sample_path(plan.size());
    std::transform(plan.begin(), plan.end(), sample_path.begin(), [&](const PathPoint &pose) {
        tf::Pose w_pose = robot_to_path_transform_ * tf::Pose(tf::createQuaternionFromYaw(pose.pose.theta),
                                                              tf::Vector3(pose.pose.x, pose.pose.y, 0));
        return tf::Point(w_pose.getOrigin().x(), w_pose.getOrigin().y(), 0);
    });
    // get the global segment
    std::vector<tf::Point> global_segment = GlobalPath::get_global_segment(sample_path);

    // use dtw to comp sample to global
    int window = (int)(dtw_window_factor_ * std::max(global_segment.size(), sample_path.size()));
    double dtw_val = GlobalPath::dtw_distance(global_segment, sample_path, window);

    // for publishing / debugging
    if (viz) {
        nav_msgs::Path global_seg_msg;  // convert type
        for (const auto path_point : global_segment) {
            geometry_msgs::PoseStamped ps;
            ps.pose.position.x = path_point.getX();
            ps.pose.position.y = path_point.getY();
            global_seg_msg.poses.push_back(ps);
        }
        global_seg_msg.header.frame_id = "map";
        global_path_seg_pub_.publish(global_seg_msg);
    }
    return dtw_val;
}

int GlobalPath::closest_index_to_pt(const std::vector<tf::Point>& path, const tf::Point& pt) {
    std::vector<double> distances_to_pt(path.size());
    std::transform(path.begin(), path.end(), distances_to_pt.begin(),
                   [pt](const tf::Point& path_pt) {
                       return GlobalPath::GetPointDistance(path_pt, pt);
    });
    return std::min_element(distances_to_pt.begin(), distances_to_pt.end()) - distances_to_pt.begin();
}

std::vector<tf::Point> GlobalPath::get_path_segment(const std::vector<tf::Point>& path, int start, int end) {
    std::vector<tf::Point> global_segment;
    if (start <= end) {  // assume sample path length < global path length
        global_segment =
                std::vector<tf::Point>(global_path_.begin() + start, global_path_.begin() + end);
    } else {
        global_segment = std::vector<tf::Point>(global_path_.begin() + start, global_path_.end());
        global_segment.insert(global_segment.end(), global_path_.begin(), global_path_.begin() + end);
    }
    return global_segment;
}

double GlobalPath::GetLocalPathProgress(const std::vector<PathPoint>& plan) {
    if (!has_global_path_) {
        return 0.0;
    }
    // convert points to world frame
    std::vector<tf::Point> sample_path(plan.size());
    std::transform(plan.begin(), plan.end(), sample_path.begin(), [&](const PathPoint &pose) {
        tf::Pose w_pose = robot_to_path_transform_ * tf::Pose(tf::createQuaternionFromYaw(pose.pose.theta),
                                                              tf::Vector3(pose.pose.x, pose.pose.y, 0));
        return tf::Point(w_pose.getOrigin().x(), w_pose.getOrigin().y(), 0);
    });
    //Get the index of the closest global points to the begining and end of the sample plan
    int seg_start_index = closest_index_to_pt(global_path_, sample_path[0]);
    int seg_end_index = closest_index_to_pt(global_path_, sample_path.back());
    if (seg_start_index == seg_end_index) { //path too short / loop back on itself
        return 1.0;
    }
//    ROS_INFO("start %f,%f", sample_path[0].getX(), sample_path[0].getY());
//    ROS_INFO("end %f,%f", sample_path.back().getX(), sample_path.back().getY());
//    ROS_INFO("sample path size: %zu", sample_path.size());
//    ROS_INFO("start: %d end: %d", seg_start_index, seg_end_index);
//    ROS_INFO("Gloabl seg");
    //Get the length of the global path segment this creates.
    std::vector<tf::Point> global_segment = get_path_segment(global_path_, seg_start_index, seg_end_index);
//    ROS_INFO("adj dists ");
//    std::vector<double> sample_adj_dist = GlobalPath::adjacent_distances(global_segment);
    std::vector<double> sample_adj_dist(1, 0.0);
//    ROS_INFO("transform");
    std::transform(global_segment.begin(), global_segment.end() - 1, global_segment.begin() + 1, std::back_inserter(sample_adj_dist),
                   &GlobalPath::GetPointDistance);
//    ROS_INFO("size, %zu", global_segment.size());
//    for (u_int i = 0; i < global_segment.size() - 1; i++) {
//        double size = global_segment[i].distance(global_segment[i+1]);
//        sample_adj_dist.push_back(size);
//    }
//    ROS_INFO("plan len");
    double plan_len = std::accumulate(sample_adj_dist.begin(), sample_adj_dist.end(), 0.0);
//    ROS_INFO("normalizing");
    //normalize and invert so shorter paths give higher costs
    double global_path_percent = plan_len / global_path_len_; // assumes all paths are shorter than the global path
    return 1.0 - global_path_percent;
}

std::vector<tf::Point> GlobalPath::get_global_segment(const std::vector<tf::Point> &sample_path) {
    // get a list of each of the global points distances from the origin of the sample path
    tf::Point sample_origin = sample_path[0];
    std::vector<double> distances_to_origin(global_path_.size());
    std::transform(global_path_.begin(), global_path_.end(), distances_to_origin.begin(),
                   [sample_origin](const tf::Point &global_pnt) {
                       return GlobalPath::GetPointDistance(global_pnt, sample_origin);
                   });
    // get the index of the closest global point to the start of the sample path
    int seg_start_index =
          std::min_element(distances_to_origin.begin(), distances_to_origin.end()) - distances_to_origin.begin();

    // get the length of sample path by summing all the dists btwn the points
    std::vector<double> sample_adj_dist = GlobalPath::adjacent_distances(sample_path);
    double sample_length = std::accumulate(sample_adj_dist.begin(), sample_adj_dist.end(), 0.0);
    // get the ending point of the global segment using the length of the segment
    double upper_cum_limit =
          std::fmod((global_cum_dist_[seg_start_index] + sample_length), global_cum_dist_[global_cum_dist_.size() - 1]);
    int seg_end_index = std::upper_bound(global_cum_dist_.begin(), global_cum_dist_.end(), upper_cum_limit) -
                        global_cum_dist_.begin();

    // create the global segment
    std::vector<tf::Point> global_segment;
    if (seg_start_index <= seg_end_index) {  // assume sample path length < global path length
        global_segment =
              std::vector<tf::Point>(global_path_.begin() + seg_start_index, global_path_.begin() + seg_end_index);
    } else {
        global_segment = std::vector<tf::Point>(global_path_.begin() + seg_start_index, global_path_.end());
        global_segment.insert(global_segment.end(), global_path_.begin(), global_path_.begin() + seg_end_index);
    }
    return global_segment;
}

// follows the pseudocode found in https://en.wikipedia.org/wiki/Dynamic_time_warping
double GlobalPath::dtw_distance(const std::vector<tf::Point> &path1, const std::vector<tf::Point> &path2, int w) {
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

std::vector<double> GlobalPath::adjacent_distances(const std::vector<tf::Point> &path) {
    std::vector<double> distances(1, 0.0);
    std::transform(path.begin(), path.end() - 1, path.begin() + 1, std::back_inserter(distances),
                   &GlobalPath::GetPointDistance);
    return distances;
}

void GlobalPath::PreProcess() {
    if (!has_global_path_) {
        return;
    }
    this->LookupPathTransform();
}

void GlobalPath::LookupPathTransform() {
    if (has_global_path_) {
        try {
            listener_->waitForTransform(global_path_msg_.header.frame_id, robot_base_frame_, ros::Time(0),
                                        ros::Duration(.05));
            listener_->lookupTransform(global_path_msg_.header.frame_id, robot_base_frame_, ros::Time(0),
                                       robot_to_path_transform_);
        } catch (tf::TransformException &ex) {
            ROS_ERROR_STREAM(ex.what());
        }
    }
}

double GlobalPath::GetPointDistance(tf::Point point1, tf::Point point2) {
    return point1.distance(point2);
}

void GlobalPath::SetPathMessage(const nav_msgs::Path &global_path_msg) {
    has_global_path_ = true;
    global_path_msg_ = nav_msgs::Path(global_path_msg);
    global_path_ = std::vector<tf::Point>(global_path_msg.poses.size());
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
    global_path_len_ = std::accumulate(global_cum_dist_.begin(), global_cum_dist_.end(), 0.0);
}


}  // namespace rr
