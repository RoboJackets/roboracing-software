#pragma once

#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <tuple>

#include "planner_types.hpp"

namespace rr {

class GlobalPath {
  public:
    explicit GlobalPath(ros::NodeHandle nh);

    double CalculateCost(const std::vector<PathPoint>& plan);
    double GetLocalPathProgress(const std::vector<PathPoint>& plan, bool viz);
    void PreProcess();
    void vizDTWSegment(const std::vector<PathPoint>& plan);
    void vizProgressSegment(const std::vector<PathPoint>& plan);
    static double GetPointDistance(const tf::Point& point1, const tf::Point& point2);

  private:
    void SetPathMessage(const nav_msgs::Path& map_msg);
    std::vector<double> adjacentDistances(const std::vector<tf::Point>& path);
    std::vector<tf::Point> getGlobalSegment(const std::vector<tf::Point>& sample_path);
    double dtwDistance(const std::vector<tf::Point>& path1, const std::vector<tf::Point>& path2, int w);
    int closestPt(const std::vector<tf::Point>& path, const tf::Point& pt);
    int ptAfterDist(const std::vector<double>& cum_dist, int start, double len);
    std::vector<tf::Point> getPathSegment(const std::vector<tf::Point>& path, int start, int end);
    std::vector<tf::Point> convertToWorldPoints(const std::vector<PathPoint>& plan);

    bool has_global_path_;
    double dtw_window_factor_;
    double progress_seg_factor_;

    ros::Subscriber global_path_sub_;
    ros::Publisher global_path_seg_pub_;
    ros::Publisher progress_seg_pub_;
    std::string robot_base_frame_;
    std::string global_path_frame_;
    std::vector<tf::Point> global_path_;
    std::vector<double> global_cum_dist_;
    double global_path_len_;
    std::unique_ptr<tf::TransformListener> listener_;
    tf::StampedTransform robot_to_path_transform_;
};

}  // namespace rr
