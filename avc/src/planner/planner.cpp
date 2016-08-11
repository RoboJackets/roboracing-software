#include "planner.h"

planner::planner() {
	ros::NodeHandle nh;
}

planner::pose planner::calculateStep(double x, double y, double theta, double velocity, double steer_angle, double timestep) {
	this->velocity = velocity;
	double turn_radius = constants::wheel_base / sin(std::abs(steer_angle) * PI / 180.0);
	double temp_theta = velocity * timestep / turn_radius;
	double deltaX = turn_radius * cos(PI / 2 - temp_theta);
	double deltaY;
	if (steer_angle < 0) {
		deltaY = turn_radius - turn_radius * sin(PI / 2 - temp_theta);
	} else {
		deltaY = -(turn_radius - turn_radius * sin(PI / 2 - temp_theta));
	}
	double deltaTheta = this->velocity / constants::wheel_base * sin(steer_angle * PI / 180.0) * 180 / PI * timestep;
	pose p;
	p.x = x + (deltaX * cos(theta * PI / 180.0) - deltaY * sin(theta * PI / 180.0));
	p.y = y + (deltaX * sin(theta * PI / 180.0) + deltaY * cos(theta * PI / 180.0));
	p.theta = theta + -deltaTheta;
	return p;
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "planner");
	planner plan();
	ros::spin();
	return 0;
}