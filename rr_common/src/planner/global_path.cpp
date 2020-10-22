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
#include <numeric>
#include <rr_common/planning/vectordtw.h>



namespace rr {

GlobalPath::GlobalPath(ros::NodeHandle nh) : has_global_path_(false), accepting_updates_(true), closest_point_to_robot_index_(0), listener_(new tf::TransformListener) {
    std::string global_path_topic;
    assertions::getParam(nh, "global_path_topic", global_path_topic);
    assertions::getParam(nh, "robot_base_frame", robot_base_frame_);

    assertions::getParam(nh, "cost_scaling_factor", cost_scaling_factor_, { assertions::greater_eq(0.0) });

    global_path_sub_ = nh.subscribe(global_path_topic, 1, &GlobalPath::SetPathMessage, this);
}

double GlobalPath::CalculateCost(const std::vector<PathPoint>& plan) {
    std::vector<tf::Point> sample_path;
    std::transform(plan.begin(), plan.end(), sample_path.begin(), [](const PathPoint &pose) {return tf::Point(pose.pose.x, pose.pose.y, 0);});
    // Find nearst point for plan[0] to global path
    tf::Point sample_start = sample_path[0];
    std::vector<double> distances_to_origin;
    std::transform(global_path_.begin(), global_path_.end(), distances_to_origin.begin(), [sample_start](const tf::Point& global_pnt){
        return global_pnt.distance(sample_start);
    });
    //segement start
    int seg_start_index = std::distance(distances_to_origin.begin(), std::min_element(distances_to_origin.begin(), distances_to_origin.end()));
    std::vector<double> sample_adj_dist (sample_path.size());
    for (u_int i = 1; i < sample_path.size() - 1; i++) sample_adj_dist[i] = sample_path[i + 1].distance(sample_path[i]);
//    std::adjacent_difference(sample_path.begin(), sample_path.end(), sample_adj_dist.begin(), [](const tf::Point &p1, const tf::Point &p2) {
//        return p2.distance(p1);
//    });
    sample_adj_dist[0] = 0.0;
    double sample_length = std::accumulate(sample_adj_dist.begin(), sample_adj_dist.end(), 0.0);

    //find last point using cumulative distance
    double upper_cum_limit = std::fmod((global_cum_dist_[seg_start_index] + sample_length), global_cum_dist_[global_cum_dist_.size() - 1]);
    int seg_end_index = std::upper_bound(global_cum_dist_.begin(), global_cum_dist_.end(), upper_cum_limit) - global_cum_dist_.begin();

    //Assume sample_path.length < global_path.length
    std::vector<tf::Point> global_segment;
    if (seg_start_index <= seg_end_index) {
        global_segment = std::vector<tf::Point>(global_path_.begin() + seg_start_index, global_path_.begin() + seg_end_index);
    } else {
        global_segment = std::vector<tf::Point>(global_path_.begin() + seg_start_index, global_path_.end());
        global_segment.insert(global_segment.end(), global_path_.begin(), global_path_.begin() + seg_end_index);
    }
    // Create Point lists for the dtw
    vector<Point> global_points;
    std::transform(global_segment.begin(), global_segment.end(), global_points.begin(), [](const tf::Point& tf_pt) {
        return Point(tf_pt.getX(), tf_pt.getY());
    });
    vector<Point> sample_points;
    std::transform(sample_path.begin(), sample_path.end(), sample_points.begin(), [](const tf::Point& tf_pt) {
        return Point(tf_pt.getX(), tf_pt.getY());
    });

    // dtw
    VectorDTW dtw1(global_points.size(), 1);

    return dtw1.fastdynamic(global_points, sample_points);
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
    std::vector<double> adj_dist (global_path_.size()); //look adjacent_distancesinto
    for (u_int i = 1; i < global_path_.size() - 1; i++) adj_dist[i] = global_path_[i + 1].distance(global_path_[i]);
//    std::adjacent_difference(global_path_.begin(), global_path_.end(), adj_dist.begin(), [](const tf::Point &p1, const tf::Point &p2) {
//       return p2.distance(p1);
//    });
    adj_dist[0] = 0.0;
    // Get Cumalitive Distance
    global_cum_dist_ = std::vector<double>(adj_dist.size());
    std::partial_sum(adj_dist.begin(), adj_dist.end(), global_cum_dist_.begin());

    updated_ = true;
}

}  // namespace rr
