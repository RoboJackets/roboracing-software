#include "planner.h"

using namespace std;

planner::planner() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    string obstacle_cloud_topic;

    pnh.getParam("steer_stddev", STEER_STDDEV);
    pnh.getParam("max_steer_angle", MAX_STEER_ANGLE);
    pnh.getParam("max_speed", MAX_SPEED);
    pnh.getParam("path_stage_time", PATH_STAGE_TIME);
    pnh.getParam("path_stages", PATH_STAGES);
    pnh.getParam("collision_radius", COLLISION_RADIUS);
    pnh.getParam("path_iterations", PATH_ITERATIONS);
    pnh.getParam("time_increment", TIME_INCREMENT);
    pnh.getParam("obstacle_cloud_topic", obstacle_cloud_topic);
    pnh.getParam("alternate_path_threshold", ALT_PATH_THRESHOLD);
    pnh.getParam("connected_path_distance", CONNECTED_PATH_DIST);

    map_sub = nh.subscribe(obstacle_cloud_topic, 1, &planner::mapCb, this);
    speed_pub = nh.advertise<rr_platform::speed>("plan/speed", 1);
    steer_pub = nh.advertise<rr_platform::steering>("plan/steering", 1);
    path_pub = nh.advertise<nav_msgs::Path>("plan/path", 1);
    steer_groups_pub = nh.advertise<std_msgs::Float32MultiArray>("steer_groups", 1);

    steering_gaussian = normal_distribution<double>(0, STEER_STDDEV);
    rand_gen = mt19937(std::random_device{}());
}

path::pose planner::calculateStep(double velocity, double steer_angle, double timestep,
                                     path::pose pStart) {
    if (abs(steer_angle) < 1e-6) {
        deltaX = velocity * timestep;
        deltaY = 0;
        deltaTheta = 0;
    } else {
        double turn_radius = constants::wheel_base / sin(abs(steer_angle));
        double temp_theta = velocity * timestep / turn_radius;
        deltaX = turn_radius * cos(M_PI / 2 - temp_theta);
        deltaY;
        if (steer_angle < 0) {
            deltaY = turn_radius - turn_radius * sin(M_PI / 2 - temp_theta);
        } else {
            deltaY = -(turn_radius - turn_radius * sin(M_PI / 2 - temp_theta));
        }
        deltaTheta = velocity / constants::wheel_base * sin(-steer_angle) * timestep;
    }
    path::pose p;
    p.x = pStart.x + (deltaX * cos(pStart.theta)
                    - deltaY * sin(pStart.theta));
    p.y = pStart.y + (deltaX * sin(pStart.theta)
                    + deltaY * cos(pStart.theta));
    p.theta = pStart.theta + deltaTheta;
    return p;
}

double planner::calculatePathCost(path::path path,
                                  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree) {
    double cost = 0.0;
    for(int i = 0; i < path.poses.size(); i++) {
        cost += costAtPose(path.poses[i], kdtree) / pow(path.speeds[i], 2);
    }
    return cost;
}

path::path planner::calculatePath(vector<float> angles) {
    path::path out;
    out.poses.resize(angles.size());
    out.speeds.resize(angles.size());
    out.speeds[0] = steeringToSpeed(angles[0]);
    out.poses[0] = calculateStep(out.speeds[0], angles[0], TIME_INCREMENT);
    for(int i = 1; i < angles.size(); i++) {
        out.speeds[i] = steeringToSpeed(angles[i]);
        out.poses[i] = calculateStep(out.speeds[i], angles[i], TIME_INCREMENT, out.poses[i-1]);
    }
    return out;
}

// eyeballed it. see https://www.desmos.com/calculator/suy8y1ylf0
double planner::steeringToSpeed(double angle) {
    return MAX_SPEED * cos(angle * 1.4706 / MAX_STEER_ANGLE);
}

double planner::costAtPose(path::pose step, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree) {
    pcl::PointXYZ searchPoint(step.x, step.y, 0);
    vector<int> pointIdxRadiusSearch(1);
    vector<float> pointRadiusSquaredDistance(1);
    int nResults = kdtree.nearestKSearch(searchPoint, 1, pointIdxRadiusSearch,
                                         pointRadiusSquaredDistance);

    double distSqr = pointRadiusSquaredDistance[0];
    double dist = sqrt(distSqr);

    if(nResults == 0) return 0; //is blind
    if(dist < COLLISION_RADIUS) return 1000.0; //collision
    return (1.0 / dist);
}

double planner::steeringSample() {
    double raw = steering_gaussian(rand_gen);
    if(raw >= -MAX_STEER_ANGLE && raw <= MAX_STEER_ANGLE) return raw;
    else return steeringSample();
}

geometry_msgs::PoseStamped planner::plannerPoseToPoseStamped(path::pose &pp) {
    geometry_msgs::PoseStamped ps;
    ps.pose.position.x = pp.x;
    ps.pose.position.y = pp.y;
    return ps;
}

void planner::mapCb(const sensor_msgs::PointCloud2ConstPtr& map) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*map, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    if(cloud->empty()) {
        ROS_WARN("Point cloud is empty");
        rr_platform::speedPtr speedMSG(new rr_platform::speed);
        rr_platform::steeringPtr steerMSG(new rr_platform::steering);
        speedMSG->speed = 0.5; //proceed with caution; E-Kill if necessary
        steerMSG->angle = 0;
        speed_pub.publish(speedMSG);
        steer_pub.publish(steerMSG);
        return;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    /* Build paths and evaluate their weights. Tracks the best weight.
     * weightedSteerVecs holds all of the random steering samples and their
     * path weights.
     */
    vector<path::WeightedSteeringVec> weightedSteerVecs(PATH_ITERATIONS);
    double angle;
    double bestWeight = 0;
    int increments = PATH_STAGE_TIME / TIME_INCREMENT;
    for(int i = 0; i < PATH_ITERATIONS; i++) {
        vector<float> steerPath(increments * PATH_STAGES, 0);
        path::WeightedSteeringVec wsv;
        for(int s = 0; s < PATH_STAGES; s++) {
            angle = steeringSample();
            wsv.steers.push_back(angle);
            fill_n(steerPath.begin() + s*increments, increments, angle);
        }

        path::path sp = calculatePath(steerPath);
        wsv.weight = 1.0 / calculatePathCost(sp, kdtree);
        weightedSteerVecs.push_back(wsv);

        if(wsv.weight > bestWeight) bestWeight = wsv.weight;
    }
    //ROS_INFO("best weight %f", bestWeight);

    // Filter paths by weight and store the results in weightedSteerVecsFiltered
    vector<path::WeightedSteeringVec> weightedSteerVecsFiltered;
    for(const path::WeightedSteeringVec &steerVec : weightedSteerVecs) {
        if(steerVec.weight > bestWeight * ALT_PATH_THRESHOLD) {
            weightedSteerVecsFiltered.push_back(steerVec);
        }
    }

    vector<path::SteeringGroup> groups;
    cluster(weightedSteerVecsFiltered, groups, CONNECTED_PATH_DIST, 0);
    ROS_INFO("Planner found %d path categories", (int)groups.size());

    // find the group with highest average weight and use its steering angles
    int bestGroupIndex = -1;
    double bestGroupAverageWeight = -1.0;
    for(int i = 0; i < groups.size(); i++) {
        double thisGroupAverageWeight = groups[i].averageWeight();
        if(thisGroupAverageWeight > bestGroupAverageWeight) {
            bestGroupAverageWeight = thisGroupAverageWeight;
            bestGroupIndex = i;
        }
    }

    //construct a best path
    vector<float> bestGroupCenter = groups[bestGroupIndex].weightedCenter();
    vector<float> bestSteering(increments * PATH_STAGES, 0);
    for(int s = 0; s < PATH_STAGES; s++) {
        fill_n(bestSteering.begin() + s*increments, increments, bestGroupCenter[s]);
    }
    path::path bestPath = calculatePath(bestSteering);

    desired_steer_angle = bestSteering[0];
    desired_velocity = bestPath.speeds[0];

    rr_platform::speedPtr speedMSG(new rr_platform::speed);
    rr_platform::steeringPtr steerMSG(new rr_platform::steering);
    speedMSG->speed = desired_velocity;
    steerMSG->angle = desired_steer_angle;
    speed_pub.publish(speedMSG);
    steer_pub.publish(steerMSG);

    nav_msgs::Path pathMsg;
    std::transform(bestPath.poses.begin(), bestPath.poses.end(),
                   back_inserter(pathMsg.poses), plannerPoseToPoseStamped);

    pathMsg.header.frame_id = "base_footprint";
    path_pub.publish(pathMsg);

    // planner_plotter additions
    if(steer_groups_pub.getNumSubscribers() > 0) {
        std_msgs::Float32MultiArray arrayMsg;
        int i = 0;
        for(int groupIndex = 0; groupIndex < groups.size(); groupIndex++) {
            for(const auto& sample : groups[groupIndex].weightedSteers) {
                arrayMsg.data.push_back((float) groupIndex);
                arrayMsg.data.push_back(sample.steers[0]);
                arrayMsg.data.push_back(sample.steers[1]);
                arrayMsg.data.push_back(sample.weight);
                i += 4;
            }
        }
        steer_groups_pub.publish(arrayMsg);
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");
    planner plan;
    ros::spin();
    return 0;
}
