#include <ros/ros.h>
#include <cmath>
#include <random>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
//#include <std_msgs/Float32MultiArray.h>
#include "flann/flann.hpp"
#include <chrono>

using namespace std;

int N_PATH_SEGMENTS = 3;
int N_CONTROL_SAMPLES = 200;
float SEGMENT_DISTANCE[] = {2, 3, 4};
float STEER_LIMIT[] = {0.3, 0.3, 0.2};
float STEER_STDDEV[] = {0.15, 0.1, 0.1};
float DISTANCE_INCREMENT = 0.2;
float MAX_SPEED = 1.5;
float WHEEL_BASE = 0.37;
float COLLISION_RADIUS = 0.3;
float COLLISION_PENALTY = 1000.0;
float PATH_SIMILARITY_CUTOFF = 0.1;
float MAX_RELATIVE_COST = 2.0;

vector<normal_distribution<float>> steering_gaussians;
mt19937 rand_gen;
ros::Publisher speed_pub, steer_pub, path_pub;

typedef vector<float> control_vector;

struct Pose2D {
    float x;
    float y;
    float theta;
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
        if (steerAngle < 0) {
            deltaY = turnRadius - turnRadius * sin(M_PI / 2 - tempTheta);
        } else {
            deltaY = -(turnRadius - turnRadius * sin(M_PI / 2 - tempTheta));
        }
        deltaTheta = DISTANCE_INCREMENT / WHEEL_BASE * sin(-steerAngle);
    }

    inOutPose.x += deltaX * cos(inOutPose.theta) - deltaY * sin(inOutPose.theta);
    inOutPose.y += deltaX * sin(inOutPose.theta) + deltaY * cos(inOutPose.theta);
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
                  (int)controlVector.size(), N_PATH_SEGMENTS);
    }

    Pose2D pose{0,0,0};
    float costSum = 0.0;

    for(int segmentIdx = 0; segmentIdx < N_PATH_SEGMENTS; segmentIdx++) {
        float steering = controlVector[segmentIdx];
        float speed = steeringToSpeed(steering, STEER_LIMIT[segmentIdx]);
        for(float dist = 0.0;
            dist < SEGMENT_DISTANCE[segmentIdx];
            dist += DISTANCE_INCREMENT)
        {
            advanceStep(steering, pose);
            costSum += getCostAtPose(pose, kdtree) / speed;
        }
    }
    return costSum;
}

float steeringSample(int stage) {
    float raw = steering_gaussians[stage](rand_gen);
    if(raw >= -STEER_LIMIT[stage] && raw <= STEER_LIMIT[stage])
        return raw;
    else
        return steeringSample(stage);
}

void getLocalMinima(const vector<control_vector> &controlVectors,
                    const vector<float> &costs,
                    vector<int> &localMinimaIndices) {

    int nSamples = controlVectors.size();

    // fill flann-format sample matrix
    float sampleArray[nSamples * N_PATH_SEGMENTS];
    for(int i = 0; i < nSamples; i++) {
        for(int j = 0; j < N_PATH_SEGMENTS; j++) {
            sampleArray[i * N_PATH_SEGMENTS + j] = controlVectors[i][j];
        }
    }
    flann::Matrix<float> samples(sampleArray, nSamples, N_PATH_SEGMENTS);

    // construct FLANN index (nearest neighbors preprocessing)
    flann::Index<flann::L2<float>> flannIndex(samples, flann::KDTreeSingleIndexParams());
    flannIndex.buildIndex();

    // Initialize group membership list
    vector<int> sampleStatus(nSamples);
    int UNKNOWN = 0, KNOWN = 1;
    fill_n(sampleStatus.begin(), nSamples, UNKNOWN);

    // initialize input/output parameters for radius searches
    flann::Matrix<int> indices(new int[nSamples], 1, nSamples);
    flann::Matrix<float> dists(new float[nSamples], 1, nSamples);
    flann::Matrix<float> query(new float[N_PATH_SEGMENTS], 1, N_PATH_SEGMENTS);
    flann::SearchParams searchParams(1);
    const float searchRadius = PATH_SIMILARITY_CUTOFF;

    for(int i = 0; i < nSamples; i++) {
        if(sampleStatus[i] != UNKNOWN) continue;

        // perform radius search
        for(int d = 0; d < N_PATH_SEGMENTS; d++) {
            query[0][d] = samples[i][d];
        }
        int nNeighbors = flannIndex.radiusSearch(query, indices, dists, searchRadius, searchParams);

        bool anyBetterInArea = false;
        for(int j = 0; j < nNeighbors; j++) {
            int foundIndex = indices[0][j];
            if(foundIndex != i) { //handle index of query later
                if(costs[foundIndex] < costs[i]) {
                    anyBetterInArea = true;
                } else {
                    sampleStatus[foundIndex] = KNOWN; //known not minimum
                }
            }
        }
//        cout << "i " << i << ", nNeighbors " << nNeighbors << ", and found better "
//             << anyBetterInArea << endl;

        sampleStatus[i] = KNOWN;
        if(!anyBetterInArea) {
            // includes regions with only one point
//            cout << "pushing back " << i << endl;
            localMinimaIndices.push_back(i);
        }
    }
}


void mapCallback(const sensor_msgs::PointCloud2ConstPtr& map) {
    chrono::time_point<std::chrono::system_clock> start, end, subStart, subEnd;
    start = std::chrono::system_clock::now();

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*map, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    if (cloud->empty()) {
        ROS_WARN("environment map pointcloud is empty");
        rr_platform::speedPtr speedMSG(new rr_platform::speed);
        rr_platform::steeringPtr steerMSG(new rr_platform::steering);
        speedMSG->speed = MAX_SPEED / 4; //proceed with caution; E-Kill if necessary
        steerMSG->angle = 0;
        steerMSG->header.stamp = ros::Time::now();
        speedMSG->header.stamp = ros::Time::now();
        speed_pub.publish(speedMSG);
        steer_pub.publish(steerMSG);
        return;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    vector<control_vector> controlVectors(N_CONTROL_SAMPLES);
    vector<float> costs(N_CONTROL_SAMPLES);

    subStart = std::chrono::system_clock::now();
    float bestCost = 1e10;
    for(int i = 0; i < N_CONTROL_SAMPLES; i++) {
        for(int stage = 0; stage < N_PATH_SEGMENTS; stage++) {
            controlVectors[i].push_back(steeringSample(stage));
        }
        float cost = aggregateCost(controlVectors[i], kdtree);
        if(cost < bestCost)
            bestCost = cost;
        costs[i] = cost;
    }
    subEnd = std::chrono::system_clock::now();

    vector<control_vector> goodControlVectors;
    vector<float> goodCosts;
    for(int i = 0; i < N_CONTROL_SAMPLES; i++) {
        if(costs[i] < bestCost * MAX_RELATIVE_COST) {
            goodControlVectors.push_back(controlVectors[i]);
            goodCosts.push_back(costs[i]);
        }
    }

    vector<int> localMinimaIndices;

    getLocalMinima(goodControlVectors, goodCosts, localMinimaIndices);

    // TODO implement smart path selection. For now, choose the lowest cost
    int bestGoodIndex = 0;
    for(int i : localMinimaIndices) {
        if(goodCosts[i] < goodCosts[bestGoodIndex]) {
            bestGoodIndex = i;
        }
    }

    ROS_INFO("Planner found %d local minima in control space. Best cost is %.3f",
             (int)localMinimaIndices.size(), goodCosts[bestGoodIndex]);

    rr_platform::speedPtr speedMSG(new rr_platform::speed);
    rr_platform::steeringPtr steerMSG(new rr_platform::steering);
    steerMSG->angle = goodControlVectors[bestGoodIndex][0];
    speedMSG->speed = steeringToSpeed(steerMSG->angle, STEER_LIMIT[0]);
    steerMSG->header.stamp = ros::Time::now();
    speedMSG->header.stamp = ros::Time::now();
    speed_pub.publish(speedMSG);
    steer_pub.publish(steerMSG);

    if(path_pub.getNumSubscribers() > 0) {
        nav_msgs::Path pathMsg;
        Pose2D pose{0,0,0};
        control_vector &controlVector = goodControlVectors[bestGoodIndex];
        for(int segmentIdx = 0; segmentIdx < N_PATH_SEGMENTS; segmentIdx++) {
            float steering = controlVector[segmentIdx];
            float speed = steeringToSpeed(steering, STEER_LIMIT[segmentIdx]);
            for(float dist = 0.0;
                dist < SEGMENT_DISTANCE[segmentIdx];
                dist += DISTANCE_INCREMENT)
            {
                advanceStep(steering, pose);
                geometry_msgs::PoseStamped ps;
                ps.pose.position.x = pose.x;
                ps.pose.position.y = pose.y;
                pathMsg.poses.push_back(ps);
            }
        }
        pathMsg.header.frame_id = "base_footprint";
        path_pub.publish(pathMsg);
    }

    end = std::chrono::system_clock::now();
    chrono::duration<double> elapsedSeconds = end - start;
    chrono::duration<double> clusterSeconds = subEnd - subStart;
    ROS_INFO("time proportion: %f", clusterSeconds.count() / elapsedSeconds.count());
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");

    for(int i = 0; i < N_PATH_SEGMENTS; i++) {
        steering_gaussians.push_back(normal_distribution<float>(0, STEER_STDDEV[i]));
    }
    rand_gen = mt19937(std::random_device{}());

    ros::NodeHandle nh;

    auto map_sub = nh.subscribe("/map", 1, mapCallback);
    speed_pub = nh.advertise<rr_platform::speed>("plan/speed", 1);
    steer_pub = nh.advertise<rr_platform::steering>("plan/steering", 1);
    path_pub = nh.advertise<nav_msgs::Path>("plan/path", 1);

    ROS_INFO("planner initialized");

    ros::spin();
    return 0;
}
