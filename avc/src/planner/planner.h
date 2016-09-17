#include <ros/ros.h>
#include <ros/publisher.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <rr_platform_msgs/speed.h>
#include <rr_platform_msgs/steering.h>
#include <vector>
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

	const double PI = 3.1415926535;
	const double MAX_STEER_ANGLE = 30;
	const double MIN_SPEED = 0.33;
	const double MAX_SPEED = 1.0;
	const double TIMESTEP = 5.0;

	double SPEED_INCREMENT;
	double ANGLE_INCREMENT;
	double TIME_INCREMENT;
	double SEARCH_RADIUS;
	double DISTANCE_INCREMENT;

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

	pose calculateStep(double x, double y, double theta, double speed, double steer_angle, double timestep);
	double calculatePathCost(double velocity, double steer_angle, pcl::PointCloud<pcl::PointXYZ>::Ptr Map);
	int costAtPose(pose step, pcl::PointCloud<pcl::PointXYZ>::Ptr Map);
	void mapCb(const sensor_msgs::PointCloud2ConstPtr& map);

};