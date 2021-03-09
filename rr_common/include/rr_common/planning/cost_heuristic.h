#pragma once

#include <geometry_msgs/Pose.h>
#include <parameter_assertions/assertions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rr_common/planning/bicycle_model.h>
#include <rr_common/planning/distance_map.h>
#include <rr_common/planning/inflation_map.h>
#include <rr_common/planning/map_cost_interface.h>
#include <rr_common/planning/nearest_point_cache.h>

#include <rr_common/linear_tracking_filter.hpp>

namespace rr {

class CostHeuristic {
  public:
    explicit CostHeuristic(ros::NodeHandle nh);

    double getMapCost(const rr::TrajectoryRollout& rollout, std::vector<double> map_costs);
    double getSpeedCost(const rr::TrajectoryRollout& rollout, double max_speed);
    double getSteeringCost(const rr::TrajectoryRollout& rollout);
    double getAngleCost(const rr::TrajectoryRollout& rollout);
    std::vector<double> getDiagnostics(const rr::Controls<1>& controls,
                                                      rr::BicycleModel* g_vehicle_model,
                                                      rr::MapCostInterface* g_map_cost_interface,
                                                      rr::LinearTrackingFilter* g_speed_model);

  private:
    double k_map_cost_;
    double k_speed_;
    double k_steering_;
    double k_angle_;
    double collision_penalty_;
};

} //namespace rr
