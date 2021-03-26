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

    void PreProcess();
    double CalculateCost(const std::vector<PathPoint>& plan);
    void visualize_global_segment(const std::vector<PathPoint>& plan);
    static double GetPointDistance(tf::Point point1, tf::Point point2);

  private:
    void SetPathMessage(const nav_msgs::Path& map_msg);
    std::vector<double> adjacent_distances(const std::vector<tf::Point>& path);
    std::vector<tf::Point> get_global_segment(const std::vector<tf::Point>& sample_path);
    double dtw_distance(const std::vector<tf::Point>& path1, const std::vector<tf::Point>& path2, int w);
    std::vector<tf::Point> convertToWorldPoints(const std::vector<PathPoint>& plan);

    bool has_global_path_;
    double dtw_window_factor_;

    ros::Subscriber global_path_sub_;
    ros::Publisher global_path_seg_pub_;
    std::string robot_base_frame_;
    std::string global_path_frame_;
    std::vector<tf::Point> global_path_;
    std::vector<double> global_cum_dist_;
    std::unique_ptr<tf::TransformListener> listener_;
    tf::StampedTransform robot_to_path_transform_;
};

}  // namespace rr
