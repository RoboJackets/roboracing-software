#include "planner.h"

planner::planner() {
	ros::NodeHandle nh;
	nh.getParam("speed_increment", SPEED_INCREMENT);
	nh.getParam("angle_increment", ANGLE_INCREMENT);
	nh.getParam("time_increment", TIME_INCREMENT);
	nh.getParam("search_radius", SEARCH_RADIUS);
	map_sub = nh.subscribe("map", 1, &planner::mapCb, this);
	speed_pub = nh.advertise<rr_platform_msgs::speed>("speed", 1);
	steer_pub = nh.advertise<rr_platform_msgs::steering>("steering", 1);
}

planner::pose planner::calculateStep(double x, double y, double theta, double velocity, double steer_angle, double timestep) {
	if (std::abs(steer_angle) < 1e-6) {
	    deltaX = velocity * timestep;
	    deltaY = 0;
	    deltaTheta = 0;
	} else {
		double turn_radius = constants::wheel_base / sin(std::abs(steer_angle) * PI / 180.0);
		double temp_theta = velocity * timestep / turn_radius;
		deltaX = turn_radius * cos(PI / 2 - temp_theta);
		deltaY;
		if (steer_angle < 0) {
			deltaY = turn_radius - turn_radius * sin(PI / 2 - temp_theta);
		} else {
			deltaY = -(turn_radius - turn_radius * sin(PI / 2 - temp_theta));
		}
		deltaTheta = velocity / constants::wheel_base * sin(-steer_angle * PI / 180.0) * 180 / PI * timestep;
	}
	pose p;
	p.x = x + (deltaX * cos(theta * PI / 180.0) - deltaY * sin(theta * PI / 180.0));
	p.y = y + (deltaX * sin(theta * PI / 180.0) + deltaY * cos(theta * PI / 180.0));
	p.theta = theta + deltaTheta;
	return p;
}

double planner::calculatePathCost(double velocity, double steer_angle, pcl::PointCloud<pcl::PointXYZ>::Ptr Map) {
	double cost;
	for(double t = 0; t < TIMESTEP; t += TIME_INCREMENT) {
		pose step = calculateStep(0, 0, 0, velocity, steer_angle, t);
		cost += costAtPose(step, Map);
	}
	return cost;
}

int planner::costAtPose(pose step, pcl::PointCloud<pcl::PointXYZ>::Ptr Map) {
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    if(!Map->empty())
    	kdtree.setInputCloud(Map);
    pcl::PointXYZ searchPoint(step.x , step.y ,0);
    std::vector<int> pointIdxRadiusSearch;
  	std::vector<float> pointRadiusSquaredDistance;
	return kdtree.radiusSearch(searchPoint, SEARCH_RADIUS, pointIdxRadiusSearch, pointRadiusSquaredDistance);
}

void planner::mapCb(const sensor_msgs::PointCloud2ConstPtr& map) {
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*map, pcl_pc2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
	double lowest_cost = std::numeric_limits<double>::infinity();
	double best_path_speed;
	double best_path_angle;
	for (double speed = MIN_SPEED; speed <= MAX_SPEED; speed += SPEED_INCREMENT) {
		for (double angle = -MAX_STEER_ANGLE; angle <= MAX_STEER_ANGLE; angle += ANGLE_INCREMENT) {
			double cost = calculatePathCost(speed, angle, cloud);
			if(cost < lowest_cost) {
				best_path_speed = speed;
				best_path_angle = angle;
				lowest_cost = cost;
			}
		}
	}
	rr_platform_msgs::speedPtr speedMSG(new rr_platform_msgs::speed);
	rr_platform_msgs::steeringPtr steerMSG(new rr_platform_msgs::steering);
	speedMSG->speed = best_path_speed;
	steerMSG->angle = best_path_angle;
	speed_pub.publish(speedMSG);
	steer_pub.publish(steerMSG);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "planner");
	planner plan();
	ros::spin();
	return 0;
}