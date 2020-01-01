#include <rr_common/planning/distance_map.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace rr {

DistanceMap::DistanceMap(ros::NodeHandle nh)
      : distance_cost_map(), hit_box(ros::NodeHandle(nh, "hitbox")), listener(new tf::TransformListener) {
    std::string map_topic;
    assertions::getParam(nh, "map_topic", map_topic);
    assertions::getParam(nh, "publish_distance_map", publish_distance_map);

    assertions::getParam(nh, "cost_scaling_factor", cost_scaling_factor, { assertions::greater_eq(0.0), assertions::less_eq(1.0) });

    assertions::getParam(nh, "wall_inflation", wall_inflation, { assertions::greater_eq(0.0) } );
    assertions::getParam(nh, "wall_cost", wall_cost);

    map_sub = nh.subscribe(map_topic, 1, &DistanceMap::SetMapMessage, this);
    distance_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("distance_map", 1);
}

double DistanceMap::DistanceCost(const rr::Pose& pose) {
    auto[mx, my] = this->PoseToGridPosition(pose);

    if (my < 0 || mapMetaData.height <= my || mx < 0 || mapMetaData.width <= mx)
        return 0.0;

    return distance_cost_map.at<float>(my, mx);
}

std::pair<unsigned int, unsigned int> DistanceMap::PoseToGridPosition(const rr::Pose& pose) {
    tf::Pose w_pose = transform * tf::Pose(tf::createQuaternionFromYaw(0), tf::Vector3(pose.x, pose.y, 0));

    unsigned int mx = std::floor((w_pose.getOrigin().x() - mapMetaData.origin.position.x) / mapMetaData.resolution);
    unsigned int my = std::floor((w_pose.getOrigin().y() - mapMetaData.origin.position.y) / mapMetaData.resolution);

    return std::make_pair(mx, my);
}

std::vector<double> DistanceMap::DistanceCost(const std::vector<Pose>& poses) {
    std::vector<double> distance_costs(poses.size());
    std::transform(poses.begin(), poses.end(), distance_costs.begin(),
                   [this](const Pose& pose) { return this->DistanceCost(pose); });
    return distance_costs;
}

std::vector<double> DistanceMap::DistanceCost(const std::vector<PathPoint>& path_points) {
    std::vector<double> distance_costs(path_points.size());
    std::transform(path_points.begin(), path_points.end(), distance_costs.begin(),
                   [this](const PathPoint& pathPoint) { return this->DistanceCost(pathPoint.pose); });
    return distance_costs;
}



void DistanceMap::SetMapMessage(const boost::shared_ptr<nav_msgs::OccupancyGrid const>& map_msg) {
    if (!accepting_updates_) {
        return;
    }

    try {
        listener->waitForTransform(map_msg->header.frame_id, "base_footprint", ros::Time(0), ros::Duration(.05));
        listener->lookupTransform(map_msg->header.frame_id, "base_footprint", ros::Time(0), transform);
    } catch (tf::TransformException& ex) {
        ROS_ERROR_STREAM(ex.what());
    }

    mapMetaData = map_msg->info;

    // Turn occupancy grid to distance map in meters
    cv::Mat distance_map(mapMetaData.width, mapMetaData.height, CV_8UC1);
    memcpy(distance_map.data, map_msg->data.data(), map_msg->data.size()*sizeof(uint8_t));
    cv::threshold( distance_map, distance_map, 99, 1, CV_THRESH_BINARY_INV);

    cv::distanceTransform(distance_map, distance_map, CV_DIST_L2, 3, CV_32F);
    distance_map *= mapMetaData.resolution;


    // Convert distance map to cost map based on: 100 e^(-distance * cost_scaling_factor)
    cv::exp(-distance_map * cost_scaling_factor, distance_cost_map);
    distance_cost_map *= 100;
    distance_cost_map.setTo(-1.0, distance_map < wall_inflation );

    auto[mx1, my1] = this->PoseToGridPosition(rr::Pose(hit_box.min_x, hit_box.min_y, 0));
    auto[mx2, my2] = this->PoseToGridPosition(rr::Pose(hit_box.min_x, hit_box.max_y, 0));
    auto[mx3, my3] = this->PoseToGridPosition(rr::Pose(hit_box.max_x, hit_box.max_y, 0));
    auto[mx4, my4] = this->PoseToGridPosition(rr::Pose(hit_box.max_x, hit_box.min_y, 0));

    std::vector<cv::Point> x{cv::Point(mx1, my1), cv::Point(mx2, my2), cv::Point(mx3, my3), cv::Point(mx4, my4),};
    cv::fillConvexPoly(distance_cost_map, x, cv::Scalar(0));


    if (publish_distance_map && distance_map_pub.getNumSubscribers() > 0) {
        nav_msgs::OccupancyGrid occupancyGrid;
        occupancyGrid.info = mapMetaData;
        occupancyGrid.data = std::vector<int8_t>(mapMetaData.height * mapMetaData.width, 0);

        cv::Mat distance_cost_map_int8;
        distance_cost_map.convertTo(distance_cost_map_int8, CV_8SC1);
        distance_cost_map_int8.setTo(-80, distance_cost_map < 0);

        occupancyGrid.data.assign(distance_cost_map_int8.data, distance_cost_map_int8.data + distance_cost_map_int8.total());

//        for (unsigned int r = 0; r < map_msg->info.width; r++) {
//            for (unsigned int c = 0; c < map_msg->info.width; c++) {
//                double distance = distance_map.at<float>(r, c) * mapMetaData.resolution;
//                if (distance < .5)
//                    occupancyGrid.data[r * map_msg->info.width + c] = -80;
//                else
//                    occupancyGrid.data[r * map_msg->info.width + c] = floor(100 * exp(-distance * .6));
//            }
//        }

        distance_map_pub.publish(occupancyGrid);
    }



    updated_ = true;
}
}  // namespace rr
