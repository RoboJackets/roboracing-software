#include <rr_common/planning/distance_map.h>

namespace rr {

DistanceMap::DistanceMap(ros::NodeHandle nh)
      : distance_cost_map(), hit_box(ros::NodeHandle(nh, "hitbox")), listener(new tf::TransformListener) {
    std::string map_topic;
    assertions::getParam(nh, "map_topic", map_topic);
    assertions::getParam(nh, "publish_distance_map", publish_distance_map);
    assertions::getParam(nh, "publish_inscribed_circle", publish_inscribed_circle);

    assertions::getParam(nh, "cost_scaling_factor", cost_scaling_factor, { assertions::greater_eq(0.0) });
    assertions::getParam(nh, "wall_inflation", wall_inflation, { assertions::greater_eq(0.0) });

    map_sub = nh.subscribe(map_topic, 1, &DistanceMap::SetMapMessage, this);
    distance_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("distance_map", 1);
    inscribed_circle_pub = nh.advertise<geometry_msgs::PolygonStamped>("inscribed_circle", 1);

    std::tie(inscribed_circle_radius, inscribed_circle_origin) = hit_box.getForwardInscribedCircle();
}

double DistanceMap::DistanceCost(const rr::Pose& pose) {
    auto [mx, my] = this->PoseToGridPosition(pose);

    if (my < 0 || mapMetaData.height <= my || mx < 0 || mapMetaData.width <= mx)
        return 0.0;

    return distance_cost_map.at<float>(my, mx);
}

std::pair<unsigned int, unsigned int> DistanceMap::PoseToGridPosition(const rr::Pose& pose) {
    tf::Pose w_pose = transform * tf::Pose(tf::createQuaternionFromYaw(0),
                                           tf::Vector3(pose.x + inscribed_circle_origin, pose.y, 0));

    unsigned int mx = std::floor((w_pose.getOrigin().x() - mapMetaData.origin.position.x) / mapMetaData.resolution);
    unsigned int my = std::floor((w_pose.getOrigin().y() - mapMetaData.origin.position.y) / mapMetaData.resolution);

    return std::make_pair(mx, my);
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
    memcpy(distance_map.data, map_msg->data.data(), map_msg->data.size() * sizeof(uint8_t));
    cv::threshold(distance_map, distance_map, 99, 1, CV_THRESH_BINARY_INV);

    cv::distanceTransform(distance_map, distance_map, CV_DIST_L2, 3, CV_32F);
    distance_map *= mapMetaData.resolution;

    // Convert distance map to cost map based on: 100 * e^(-distance * cost_scaling_factor)
    cv::exp(-(distance_map - (wall_inflation + inscribed_circle_radius)) * cost_scaling_factor, distance_cost_map);
    distance_cost_map *= 100;
    distance_cost_map.setTo(-1.0, distance_map <= wall_inflation + inscribed_circle_radius);

    updated_ = true;

    if (publish_distance_map && distance_map_pub.getNumSubscribers() > 0) {
        nav_msgs::OccupancyGrid occupancyGrid;
        occupancyGrid.info = mapMetaData;
        occupancyGrid.data = std::vector<int8_t>(mapMetaData.height * mapMetaData.width, 0);

        cv::Mat distance_cost_map_int8;
        distance_cost_map.convertTo(distance_cost_map_int8, CV_8SC1);
        distance_cost_map_int8.setTo(-10, distance_map < wall_inflation + inscribed_circle_radius);
        distance_cost_map_int8.setTo(-80, distance_map < wall_inflation);

        occupancyGrid.data.assign(distance_cost_map_int8.data,
                                  distance_cost_map_int8.data + distance_cost_map_int8.total());

        distance_map_pub.publish(occupancyGrid);
    }

    if (publish_inscribed_circle && inscribed_circle_pub.getNumSubscribers() > 0) {
        geometry_msgs::PolygonStamped circle;
        circle.polygon.points = std::vector<geometry_msgs::Point32>(16);
        circle.header.frame_id = map_msg->header.frame_id;
        tf::Pose w_pose =
              transform * tf::Pose(tf::createQuaternionFromYaw(0), tf::Vector3(inscribed_circle_origin, 0, 0));

        for (unsigned int i = 0; i < circle.polygon.points.size(); i++) {
            double angle = i * 2 * M_PI / circle.polygon.points.size();
            circle.polygon.points[i].x = w_pose.getOrigin().x() + inscribed_circle_radius * cos(angle);
            circle.polygon.points[i].y = w_pose.getOrigin().y() + inscribed_circle_radius * sin(angle);
        }

        inscribed_circle_pub.publish(circle);
    }
}

}  // namespace rr
