#ifndef RR_COMMON_PLANNER_H
#define RR_COMMON_PLANNER_H

#include <cmath>
#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <avc/constants.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <random>

#include "path_structs.h"
#include "density_cluster.h"


class planner {
public:
	planner();

private:
	ros::Subscriber map_sub;
	ros::Publisher speed_pub;
	ros::Publisher steer_pub;
	ros::Publisher path_pub;

	std::normal_distribution<double> steering_gaussian;
	std::mt19937 rand_gen;

	double STEER_STDDEV; //standard dev of steering randomizer (degrees)
	double MAX_STEER_ANGLE; //degrees
	int PATH_ITERATIONS; //number of random paths to generate
	double MAX_SPEED; //meters per second
	double PATH_STAGE_TIME; //simulate this much time per control value
	double TIME_INCREMENT; //timestep between points on the path
	int PATH_STAGES; //number of control values per path
	double COLLISION_RADIUS; //minimum acceptable distance to obstacle
	double ALT_PATH_THRESHOLD; //proportion of the max weight needed to use a path
	double CONNECTED_PATH_DIST; //euclidean distance between two "similar enough" paths

	double deltaX;
	double deltaY;
	double deltaTheta;

	double desired_steer_angle;
	double desired_velocity;



	static geometry_msgs::PoseStamped plannerPoseToPoseStamped(path::pose &p);
    static bool steeringVecCompare(const path::WeightedSteeringVec &wsv1, const path::WeightedSteeringVec &wsv2);

	path::pose calculateStep(double speed, double steer_angle, double timestep, path::pose pStart = path::pose{0,0,0});
	double steeringToSpeed(double angle);
	double steeringSample();
	path::path calculatePath(std::vector<double> angles);
	double calculatePathCost(path::path path, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree);
	double costAtPose(path::pose step, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree);
	void mapCb(const sensor_msgs::PointCloud2ConstPtr& map);

};

#endif //RR_COMMON_PLANNER_H
