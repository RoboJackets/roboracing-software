#include "planner.h"

using namespace std;

planner::planner() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.getParam("steer_stddev", STEER_STDDEV);
    pnh.getParam("max_steer_angle", MAX_STEER_ANGLE);
    pnh.getParam("max_speed", MAX_SPEED);
    pnh.getParam("path_stage_time", PATH_STAGE_TIME);
    pnh.getParam("path_stages", PATH_STAGES);
    pnh.getParam("collision_radius", COLLISION_RADIUS);
    pnh.getParam("path_iterations", PATH_ITERATIONS);
    pnh.getParam("time_increment", TIME_INCREMENT);

    map_sub = nh.subscribe("map", 1, &planner::mapCb, this);
    speed_pub = nh.advertise<rr_platform::speed>("speed", 1);
    steer_pub = nh.advertise<rr_platform::steering>("steering", 1);
    path_pub = nh.advertise<nav_msgs::Path>("path", 1);

    steering_gaussian = normal_distribution<double>(0, STEER_STDDEV);
    rand_gen = mt19937(std::random_device{}());
}

planner::pose planner::calculateStep(double velocity, double steer_angle, double timestep,
                                     planner::pose pStart) {
    if (abs(steer_angle) < 1e-6) {
        deltaX = velocity * timestep;
        deltaY = 0;
        deltaTheta = 0;
    } else {
        double turn_radius = constants::wheel_base / sin(abs(steer_angle) * M_PI / 180.0);
        double temp_theta = velocity * timestep / turn_radius;
        deltaX = turn_radius * cos(M_PI / 2 - temp_theta);
        deltaY;
        if (steer_angle < 0) {
            deltaY = turn_radius - turn_radius * sin(M_PI / 2 - temp_theta);
        } else {
            deltaY = -(turn_radius - turn_radius * sin(M_PI / 2 - temp_theta));
        }
        deltaTheta = velocity / constants::wheel_base * sin(-steer_angle * M_PI / 180.0) * 180 / M_PI * timestep;
    }
    pose p;
    p.x = pStart.x + (deltaX * cos(pStart.theta * M_PI / 180.0) 
                    - deltaY * sin(pStart.theta * M_PI / 180.0));
    p.y = pStart.y + (deltaX * sin(pStart.theta * M_PI / 180.0) 
                    + deltaY * cos(pStart.theta * M_PI / 180.0));
    p.theta = pStart.theta + deltaTheta;
    return p;
}

double planner::calculatePathCost(planner::sim_path path,
                                  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree) {
    double cost = 0.0;
    for(int i = 0; i < path.poses.size(); i++) {
        cost += costAtPose(path.poses[i], kdtree) / pow(path.speeds[i], 3);
    }
    return cost;
}

planner::sim_path planner::calculatePath(vector<double> angles) {
    sim_path out;
    out.angles = angles;
    out.poses.resize(angles.size());
    out.speeds.resize(angles.size());
    out.speeds[0] = steeringToSpeed(angles[0]);
    out.poses[0] = calculateStep(out.speeds[0], angles[0], TIME_INCREMENT);
    for(int i = 1; i < angles.size(); i++) {
        out.speeds[i] = steeringToSpeed(angles[i]);
        out.poses[i] = calculateStep(out.speeds[i], angles[i], TIME_INCREMENT, out.poses[i-1]);
    }
    // ROS_INFO("path used %d steps", nSteps);
    return out;
}

// eyeballed it. see https://www.desmos.com/calculator/qv1jpaeyuz
double planner::steeringToSpeed(double angle) {
    return MAX_SPEED * cos(angle * M_PI * 0.4681 / MAX_STEER_ANGLE);
}

double planner::costAtPose(pose step, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree) {
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

geometry_msgs::PoseStamped planner::plannerPoseToPoseStamped(planner::pose pp) {
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

    //build paths and evaluate their weights
    vector<double> weightedSteeringRaw;
    double weightTotal = 0, cost, angle;
    int increments = PATH_STAGE_TIME / TIME_INCREMENT;

    for(int i = 0; i < PATH_ITERATIONS; i++) {
        vector<double> steerPath(increments * PATH_STAGES, 0);
        for(int s = 0; s < PATH_STAGES; s++) {
            angle = steeringSample();
            fill_n(steerPath.begin() + s*increments, increments, angle);
        }

        sim_path sp = calculatePath(steerPath);
        cost = calculatePathCost(sp, kdtree);
        weightTotal += 1.0 / cost;

        for(int j = 0; j < sp.angles.size(); j++) {
            if(j < weightedSteeringRaw.size()) {
                weightedSteeringRaw[j] += sp.angles[j] / cost;
            } else {
                weightedSteeringRaw.push_back(sp.angles[j] / cost);
            }
        }
    }

    vector<double> bestSteering;
    for(int i = 0; i < weightedSteeringRaw.size(); i++) {
        bestSteering.push_back(weightedSteeringRaw[i] / weightTotal);
    }

    sim_path bestPath = calculatePath(bestSteering);

    desired_steer_angle = bestPath.angles[0];
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

    pathMsg.header.frame_id = "map";
    path_pub.publish(pathMsg);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");
    planner plan;
    ros::spin();
    return 0;
}
