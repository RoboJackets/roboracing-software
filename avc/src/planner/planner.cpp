#include "planner.h"
#define _USE_MATH_DEFINES //for cmath
#include <cmath>

using namespace std;

planner::planner() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.getParam("steer_stddev", STEER_STDDEV);
    pnh.getParam("max_steer_angle", MAX_STEER_ANGLE);
    pnh.getParam("max_speed", MAX_SPEED);
    pnh.getParam("path_time", PATH_TIME);
    pnh.getParam("time_increment", TIME_INCREMENT);
    pnh.getParam("collision_radius", COLLISION_RADIUS);
    pnh.getParam("path_iterations", PATH_ITERATIONS);
    map_sub = nh.subscribe("map", 1, &planner::mapCb, this);
    speed_pub = nh.advertise<rr_platform::speed>("speed", 1);
    steer_pub = nh.advertise<rr_platform::steering>("steering", 1);
    path_pub = nh.advertise<nav_msgs::Path>("path", 1);
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
planner::pose planner::calculateStep(double velocity, double steer_angle, double timestep) {
    pose p;
    p.x = 0;
    p.y = 0;
    p.theta = 0;
    return calculateStep(velocity, steer_angle, timestep, p);
}

double planner::calculatePathCost(planner::sim_path path,
                                  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree) {
    double cost = 0.0;
    for(int i = 0; i < path.poses.size(); i++) {
        cost += costAtPose(path.poses[i], kdtree) / pow(path.speeds[i], 2);
    }
    return cost;
}

planner::sim_path planner::calculatePath(vector<double> angles) {
    sim_path out;
    out.initAngle = angles[0];
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

// eyeballed it. see https://www.desmos.com/calculator/hhxmjjanw1
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
    if(dist < COLLISION_RADIUS) return 100.0; //collision
    return (1.0 / distSqr);
}

double planner::steeringSample() {
    double raw = (*steering_gaussian_ptr)(*rand_gen_ptr);
    if(raw >= -MAX_STEER_ANGLE && raw <= MAX_STEER_ANGLE) return raw;
    else return steeringSample();
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
        speedMSG->speed = 0.5; //cautious
        steerMSG->angle = 0;
        speed_pub.publish(speedMSG);
        steer_pub.publish(steerMSG);
        return;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    //vector<sim_path> simPaths(PATH_ITERATIONS);

    //build paths and evaluate their weights
    double weightTotal, weightedAngleRaw, weight;
    weightTotal = weightedAngleRaw = 0;
    for(int i = 0; i < PATH_ITERATIONS; i++) {
        vector<double> steerPath;
        for(double t = 0; t < PATH_TIME; t += TIME_INCREMENT) {
            steerPath.push_back(steeringSample());
        }
        //sim_path sp = simPaths[i] = calculatePath(steerPath);
        sim_path sp = calculatePath(steerPath);
        weight = 1.0 / calculatePathCost(sp, kdtree);
        weightTotal += weight;
        weightedAngleRaw += weight * sp.initAngle;
    }

    ROS_INFO("%d", PATH_ITERATIONS);

    double best_path_angle = weightedAngleRaw / weightTotal;
    double best_path_speed = steeringToSpeed(best_path_angle);

    rr_platform::speedPtr speedMSG(new rr_platform::speed);
    rr_platform::steeringPtr steerMSG(new rr_platform::steering);
    speedMSG->speed = best_path_speed;
    steerMSG->angle = best_path_angle;
    speed_pub.publish(speedMSG);
    steer_pub.publish(steerMSG);

    nav_msgs::Path path;
    //ROS_INFO("best speed: %f", best_path_speed);
    for(double t = 0; t < PATH_TIME; t += TIME_INCREMENT) {
        pose step = calculateStep(best_path_speed, best_path_angle, t);
        geometry_msgs::PoseStamped p;
        p.pose.position.x = step.x;
        p.pose.position.y = step.y;
        path.poses.push_back(p);
    }
    path.header.frame_id = "ground";
    //ROS_INFO_STREAM("path size " << path.poses.size());
    path_pub.publish(path);

    //ROS_INFO("tried %d trajectories", nTraj);
}

//a convenient scope
void planner::spin() {
    normal_distribution<double> nd(0, STEER_STDDEV);
    steering_gaussian_ptr = &nd;
    default_random_engine dre;
    rand_gen_ptr = &dre;

    ros::spin();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");
    //ROS_INFO("hi");
    planner plan;
    //ROS_INFO("bye");
    plan.spin();
    return 0;
}
