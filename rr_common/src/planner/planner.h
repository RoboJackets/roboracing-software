#ifndef RR_COMMON_PLANNER_H
#define RR_COMMON_PLANNER_H

#include <ros/ros.h>
#include <cmath>
#include <cstring>
#include <string>
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
#include "flann/flann.hpp"

int N_PATH_SEGMENTS;
int N_CONTROL_SAMPLES;
int SMOOTHING_ARRAY_SIZE;
std::vector<float> SEGMENT_DISTANCES;
std::vector<float> STEER_LIMITS;
std::vector<float> STEER_STDDEVS;
std::vector<float> PREV_STEERING_ANGLES;
int PREV_STEERING_ANGLES_INDEX;
float DISTANCE_INCREMENT;
float MAX_SPEED;
float WHEEL_BASE;
float COLLISION_RADIUS;
float COLLISION_PENALTY;
float PATH_SIMILARITY_CUTOFF;
float MAX_RELATIVE_COST;

std::vector<std::normal_distribution<float> > steering_gaussians;
std::mt19937 rand_gen;
ros::Publisher speed_pub, steer_pub, path_pub;
std::unique_ptr<tf::TransformListener> tfListener;

typedef std::vector<float> control_vector;

struct Pose2D {
    float x;
    float y;
    float theta;
};

void advanceStep(float steerAngle, Pose2D &inOutPose);
float getCostAtPose(Pose2D &pose, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree);
inline float steeringToSpeed(float steering, float maxSteering);
float aggregateCost(control_vector &controlVector, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree);
float steeringSample(int stage);
void getLocalMinima(const std::vector<control_vector>&, const std::vector<float>&, std::vector<int>&);
void mapCallback(const sensor_msgs::PointCloud2ConstPtr& map);
void parseFloatArrayStr(std::string &arrayAsString, std::vector<float> &floats);

#endif //RR_COMMON_PLANNER_H
