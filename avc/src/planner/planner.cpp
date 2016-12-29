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

double planner::calculatePathCost(double steer_angle, 
								  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree) {
	double distCost = 0.0;
	double velocity = steeringToSpeed(steer_angle);
	for(double t = 0; t < PATH_TIME; t += TIME_INCREMENT) {
		pose step = calculateStep(velocity, steer_angle, t);
		distCost += costAtPose(step, kdtree);
		//nSteps += 1;
	}
	// ROS_INFO("path used %d steps", nSteps);
	return distCost / pow(velocity, 2);
}

// eyeballed it. see https://www.desmos.com/calculator/hhxmjjanw1
double planner::steeringToSpeed(double angle) {
	return MAX_SPEED * cos(angle * M_PI * 0.4681 / MAX_STEER_ANGLE);
}

double planner::costAtPose(pose step, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree) {
	pcl::PointXYZ searchPoint(step.x , step.y ,0);
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

void planner::mapCb(const sensor_msgs::PointCloud2ConstPtr& map) {
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*map, pcl_pc2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	double lowest_cost = numeric_limits<double>::max();
	double best_path_speed = 0;
	double best_path_angle = 0;
	double cost = 0;
	for (double angle = MAX_STEER_ANGLE; angle >= -MAX_STEER_ANGLE; angle -= 1) {
		if(cloud->empty()) cost = 0;
		else cost = calculatePathCost(angle, kdtree);

		if(cost < lowest_cost) {
			//best_path_speed = speed;
			best_path_angle = angle;
			lowest_cost = cost;
		} /*else if (cost == lowest_cost) {
			if(speed > best_path_speed) {
				best_path_speed = speed;
				best_path_angle = angle;
			} else if(speed == best_path_speed && abs(angle) < abs(best_path_angle)) {
				best_path_speed = speed;
				best_path_angle = angle;
			}
		}*/
		//ROS_INFO("cost is %.4f", cost);
	}

	rr_platform::speedPtr speedMSG(new rr_platform::speed);
	rr_platform::steeringPtr steerMSG(new rr_platform::steering);
	speedMSG->speed = best_path_speed;
	steerMSG->angle = best_path_angle;
	speed_pub.publish(speedMSG);
	steer_pub.publish(steerMSG);

	nav_msgs::Path path;
	best_path_speed = steeringToSpeed(best_path_angle);
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


int main(int argc, char** argv) {
	ros::init(argc, argv, "planner");
	//ROS_INFO("hi");
	planner plan;
	//ROS_INFO("bye");
	ros::spin();
	return 0;
}