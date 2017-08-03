#include <ros/ros.h>
#include <cmath>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

using namespace std;

int N_PATH_SEGMENTS = 2;
int BINS[] = {21, 11};
float SEGMENT_DISTANCE[] = {3.0, 5.0};
float DISTANCE_INCREMENT = 0.1;
float MAX_STEERING[] = {0.4, 0.3};
float MAX_SPEED = 1.5;
float COLLISION_RADIUS = 0.3;
float COLLISION_PENALTY = 1000.0;
float WHEEL_BASE = 0.37;

typedef vector<float> control_vector;

struct Pose2D {
    float x;
    float y;
    float theta;
};

struct PathOption {
    control_vector &controlVector;
    float cost;
};

void advanceStep(float steerAngle, Pose2D &inOutPose) {
    float deltaX, deltaY, deltaTheta;
    if (abs(steerAngle) < 1e-3) {
        deltaX = DISTANCE_INCREMENT;
        deltaY = 0;
        deltaTheta = 0;
    } else {
        float turnRadius = WHEEL_BASE / sin(abs(steerAngle));
        float tempTheta = DISTANCE_INCREMENT / turnRadius;
        deltaX = turnRadius * cos(M_PI / 2 - tempTheta);
        if (steer_angle < 0) {
            deltaY = turn_radius - turn_radius * sin(M_PI / 2 - temp_theta);
        } else {
            deltaY = -(turn_radius - turn_radius * sin(M_PI / 2 - temp_theta));
        }
        deltaTheta = DISTANCE_INCREMENT / WHEEL_BASE * sin(-steer_angle);
    }

    inOutPose.x += deltaX * cos(pStart.theta) - deltaY * sin(pStart.theta);
    inOutPose.y += deltaX * sin(pStart.theta) + deltaY * cos(pStart.theta);
    inOutPose.theta += deltaTheta;
}

float getCostAtPose(Pose2D pose, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree) {
    pcl::PointXYZ searchPoint(pose.x, pose.y, 0);
    vector<int> pointIdxRadiusSearch(1);
    vector<float> pointRadiusSquaredDistance(1);
    int nResults = kdtree.nearestKSearch(searchPoint, 1, pointIdxRadiusSearch,
                                         pointRadiusSquaredDistance);

    float dist = sqrt(pointRadiusSquaredDistance[0]);
    if(nResults == 0) { //no obstacles in sight
        return 0;
    } else if(dist < COLLISION_RADIUS) { //collision
        return COLLISION_PENALTY;
    } else {
        return (1.0 / dist);
    }
}

// eyeballed it. see https://www.desmos.com/calculator/suy8y1ylf0
inline float steeringToSpeed(float steering, float maxSteering) {
    return MAX_SPEED * cos(steering * 1.4706 / maxSteering);
}

float aggregateCost(control_vector controlVector, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree) {
    if(controlVector.size() != N_PATH_SEGMENTS) {
        ROS_ERROR("control vector of dimension %d does not match %d path segments",
                  controlVector.size(), N_PATH_SEGMENTS);
    }

    Pose2D pose{0,0,0};
    float costSum = 0.0;

    for(int segmentIdx = 0; segmentIdx < N_PATH_SEGMENTS; segmentIdx++) {
        float steering = controlVector[segmentIdx];
        float speed = steeringToSpeed(steering, MAX_STEERING[segmentIdx]);
        for(float dist = 0.0;
            dist < SEGMENT_DISTANCE[segmentIdx];
            dist += DISTANCE_INCREMENT)
        {
            advanceStep(steering, &pose);
            costSum += getCostAtPose(pose, kdtree) / speed;
        }
    }

    return costSum;
}


void mapCallback(const sensor_msgs::PointCloud2ConstPtr& map) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*map, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    if (cloud->empty()) {
        ROS_WARN("environment map pointcloud is empty");
        rr_platform::speedPtr speedMSG(new rr_platform::speed);
        rr_platform::steeringPtr steerMSG(new rr_platform::steering);
        speedMSG->speed = 0.5; //proceed with caution; E-Kill if necessary
        steerMSG->angle = 0;
        steerMSG->header.stamp = ros::Time::now();
        speedMSG->header.stamp = ros::Time::now();
        speed_pub.publish(speedMSG);
        steer_pub.publish(steerMSG);
        return;
    }

    pcl::KdTreeFLANN <pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    int nPaths = 1;
    for(int i = 0; i < N_PATH_SEGMENTS; i++) {
        nPaths *= BINS[i];
    }

    vector<PathOption> pathOptions(nPaths);
    for(int i = 0; i < nPaths; i++) {
        vector<PathOption> v;
        pathOptions.push_back(v);
    }

    for(int i = 0; i < nPaths; i++) {
        float binWidth = MAX_STEERING[segmentIdx] * 2.0 / (float) BINS[segmentIdx];
        for(int segmentIdx = 0; segmentIdx < N_PATH_SEGMENTS; segmentIdx++) {
            int bin = 0;
            float steering = -MAX_STEERING[segmentIdx] + binWidth / 2.0;
            while(steering < MAX_STEERING[segmentIdx]) {
                if(pathField[segmentIdx].size() < BINS[segmentIdx]) {
                    PathOption po;
                    po.controlVector.push_back(steering);
                    pathField[segmentIdx].push_back(po);
                } else {
                    pathField[segmentIdx][bin]
                }
                steering += binWidth;
                bin++;
            }
        }
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");


    ros::spin();
    return 0;
}
