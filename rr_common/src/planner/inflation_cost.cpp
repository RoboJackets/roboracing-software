#include <rr_common/planning/inflation_cost.h>

#include <costmap_2d/layer.h>

#define COLLISION(C) ((C) > 95)

namespace rr {
    InflationCost::InflationCost(const ros::NodeHandle &nh, const CenteredBox& box) {
//        assertions::getParam(nh, "annealing_steps", params_.annealing_steps, {assertions::greater(0)});
//        assertions::getParam(nh, "acceptance_scale", params_.acceptance_scale, {assertions::greater(0.0)});
//        assertions::getParam(nh, "temperature_end", params_.temperature_end, {assertions::greater(0.0)});
        hit_box = box;
        listener = std::make_unique<tf::TransformListener>();

    }

    double InflationCost::DistanceCost(const Pose& rr_pose) {
        const tf::Pose pose(tf::createQuaternionFromYaw(rr_pose.theta), tf::Vector3(rr_pose.x, rr_pose.y, 0));
        tf::Pose world_Pose = transform * pose;

        int mx = std::floor((world_Pose.getOrigin().x() - map->info.origin.position.x) / map->info.resolution);
        int my = std::floor((world_Pose.getOrigin().y() - map->info.origin.position.y) / map->info.resolution);
        unsigned int index = my * map->info.width + mx;

        if (index < 0  && index >= map->data.size()) return 0.0;
        unsigned int cost = map->data[index];

        if ((rr_pose.y < -hit_box.left || hit_box.right < rr_pose.y ||
            rr_pose.x < -hit_box.back || hit_box.front < rr_pose.x) &&
            cost > 90) {
            return -1.0;
        }

        return cost;
    }

    std::vector<double> InflationCost::DistanceCost(const std::vector<Pose>& poses) {
        std::vector<double> distance_costs(poses.size());
        std::transform (poses.begin(), poses.end(), distance_costs.begin(), [this](const Pose& pose) {
            return this->DistanceCost(pose);
        });

        return distance_costs;
    }


    std::vector<double> InflationCost::DistanceCost(const std::vector<PathPoint>& path_points) {
        std::vector<double> distance_costs(path_points.size());
        std::transform (path_points.begin(), path_points.end(), distance_costs.begin(), [this](const PathPoint& pathPoint) {
            return this->DistanceCost(pathPoint.pose);
        });
        return distance_costs;
    }

    void InflationCost::SetMapMessage(const boost::shared_ptr<nav_msgs::OccupancyGrid const>& map_msg) {
        map = map_msg;

        try {
            listener->waitForTransform(map_msg->header.frame_id, "/base_footprint", ros::Time(0), ros::Duration(.05));
            listener->lookupTransform(map_msg->header.frame_id, "/base_footprint", ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
            ROS_ERROR_STREAM(ex.what());
        }
    }

} // namespace rr