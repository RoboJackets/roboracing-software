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


class planner {
public:
	planner();
private:
	ros::Subscriber map_sub;
	ros::Publisher speed_pub;
	ros::Publisher steer_pub;
	ros::Publisher path_pub;

	double STEER_STDDEV; //degrees
	double MAX_STEER_ANGLE; //degrees
	double MAX_SPEED; //meters per second
	double PATH_TIME;
	double TIME_INCREMENT;
	double COLLISION_RADIUS;

	double deltaX;
	double deltaY;
	double deltaTheta;

	double desired_steer_angle;
	double desired_velocity;

	struct pose
	{
		double x;
		double y;
		double theta;
	};

	pose calculateStep(double speed, double steer_angle, double timestep);
	pose calculateStep(double speed, double steer_angle, double timestep, pose pStart);
	double steeringToSpeed(double angle);
	double calculatePathCost(double steer_angle, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree);
	double costAtPose(pose step, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree);
	void mapCb(const sensor_msgs::PointCloud2ConstPtr& map);

};