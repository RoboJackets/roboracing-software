#include <ros/ros.h>
#include <ros/publisher.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <iarrc_msgs/iarrc_speed.h>
#include "path.cpp"
#include <avc/constants.hpp>


class planner {
public:
	planner();
	~planner();
private:
	ros::Subscriber speed_sub;

	const double PI = 3.1415926535;
	const double MAX_STEER_ANGLE = 30;
	const int NUMBER_PATHS = 10;
	const double timestep = 0.1;

	double velocity;
	double desired_steer_angle;
	double desired_velocity;

	struct pose
	{
		double x;
		double y;
		double theta;
	};
	pose calculateStep(double x, double y, double theta, double velocity, double steer_angle, double timestep);
	path calculateCost(const std::vector<path>);

};