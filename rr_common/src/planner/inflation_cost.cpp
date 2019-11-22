#include <rr_common/planning/inflation_cost.h>

namespace rr {

InflationCost::InflationCost(ros::NodeHandle nh)
      : map(), hit_box(ros::NodeHandle(nh, "hitbox")), listener(new tf::TransformListener) {
    std::string map_topic;
    assertions::getParam(nh, "map_topic", map_topic);
    map_sub = nh.subscribe(map_topic, 1, &InflationCost::SetMapMessage, this);

    assertions::getParam(nh, "lethal_threshold", lethal_threshold, { assertions::greater(0), assertions::less(256) });
}

double InflationCost::DistanceCost(const rr::Pose& rr_pose) {
    const tf::Pose pose(tf::createQuaternionFromYaw(rr_pose.theta), tf::Vector3(rr_pose.x, rr_pose.y, 0));
    tf::Pose world_Pose = transform * pose;

    int mx = std::floor((world_Pose.getOrigin().x() - map->info.origin.position.x) / map->info.resolution);
    int my = std::floor((world_Pose.getOrigin().y() - map->info.origin.position.y) / map->info.resolution);
    unsigned int index = my * map->info.width + mx;

    if (index < 0 && index >= map->data.size())
        return 0.0;
    char cost = map->data[index];

    if (!hit_box.PointInside(rr_pose.x, rr_pose.y) && (cost > lethal_threshold || cost < 0)) {
        return -1.0;
    }

    return cost;
}

std::vector<double> InflationCost::DistanceCost(const std::vector<Pose>& poses) {
    std::vector<double> distance_costs(poses.size());
    std::transform(poses.begin(), poses.end(), distance_costs.begin(),
                   [this](const Pose& pose) { return this->DistanceCost(pose); });
    return distance_costs;
}

std::vector<double> InflationCost::DistanceCost(const std::vector<PathPoint>& path_points) {
    std::vector<double> distance_costs(path_points.size());
    std::transform(path_points.begin(), path_points.end(), distance_costs.begin(),
                   [this](const PathPoint& pathPoint) { return this->DistanceCost(pathPoint.pose); });
    return distance_costs;
}

void InflationCost::SetMapMessage(const boost::shared_ptr<nav_msgs::OccupancyGrid const>& map_msg) {
    if (!accepting_updates_) {
        return;
    }

    map = map_msg;

    try {
        listener->waitForTransform(map_msg->header.frame_id, "/base_footprint", ros::Time(0), ros::Duration(.05));
        listener->lookupTransform(map_msg->header.frame_id, "/base_footprint", ros::Time(0), transform);
    } catch (tf::TransformException& ex) {
        ROS_ERROR_STREAM(ex.what());
    }

    updated_ = true;
}

}  // namespace rr
