#include "follower.h"
using namespace std;
//rr_msgs::speedPtr speedMSG(new rr_msgs::speed);

void pidCallback(std_msgs::Float64 pidSpeed) {
//    float newSpeed = pidSpeed;
    rr_msgs::speed outputSpeed;
    double newSpeed = (double) pidSpeed.data;
    outputSpeed.speed = newSpeed;
    speed_pub.publish(outputSpeed);

}
void mapCallback(const sensor_msgs::PointCloud2ConstPtr& map) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*map, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    rr_msgs::speedPtr speedMSG(new rr_msgs::speed);
    rr_msgs::steeringPtr steerMSG(new rr_msgs::steering);
    if (cloud->empty()) {
        ROS_WARN("environment map pointcloud is empty");
        speedMSG->speed = 0;
        steerMSG->angle = 0;
        speed_pub.publish(speedMSG);
        steer_pub.publish(steerMSG);
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Bounds the x-dimension
    pcl::PassThrough<pcl::PointXYZ> passLimitX;
    passLimitX.setInputCloud(cloud);
    passLimitX.setFilterFieldName("x");
    passLimitX.setFilterLimits(MIN_FRONT_VISION, MAX_FRONT_VISION);
    passLimitX.filter(*cloud_filtered);

    // Bounds the y-dimension
    pcl::PassThrough<pcl::PointXYZ> passLimitY;
    passLimitY.setInputCloud(cloud_filtered);
    passLimitY.setFilterFieldName("y");
    passLimitY.setFilterLimits(MIN_SIDE_VISION, MAX_SIDE_VISION);
    passLimitY.filter(*cloud_filtered);

    // avgX is the AVERAGE foward distance of all the points in the vision box
//    float avgX = 0.0f;
    float minX = INT_MAX;
    for (pcl::PointXYZ point : cloud_filtered->points) {
//        avgX += point.x;
        if (point.x < minX) {
            minX = point.x;
        }
    }

    // Uncomment to make car go GOAL_DIST from the CLOSEST object
    // avgX = minx;

    if (cloud_filtered->points.size() != 0) {  // computes the average distance away
//        avgX = avgX / cloud_filtered->points.size();
//        ROS_INFO_STREAM("Average X = " << avgX);
        ROS_INFO_STREAM("Min X = " << minX);

    } else {  // Nothing in bounding box
        ROS_INFO_STREAM("Nothing in bounding box");
        speedMSG->speed = 0;
        steerMSG->angle = 0;
        speed_pub.publish(speedMSG);
        steer_pub.publish(steerMSG);
        return;
    }

//    if (avgX <= GOAL_DIST - GOAL_MARGIN_OF_ERR && avgX >= MIN_FRONT_VISION)
//        speedMSG->speed = -(FOLLOWER_SPEED);  // when object is too close, move backward
//    else if (avgX >= GOAL_DIST + GOAL_MARGIN_OF_ERR && avgX <= MAX_FRONT_VISION)
//        speedMSG->speed = FOLLOWER_SPEED;  // when object is too far, move forward
//    else
//        speedMSG->speed = 0.0;  // when the object exceeds MAX_FRONT_VISION or
                                // another anomali occurs




    std_msgs::Float64 currDist;
//    currDist.data = avgX;
    currDist.data = minX;
    pid_speed_pub.publish(currDist);
    std_msgs::Float64 setDist;
    setDist.data = GOAL_DIST;
    pid_setpoint_pub.publish(setDist);



    steerMSG->angle = 0;
//    speed_pub.publish(speedMSG);
    steer_pub.publish(steerMSG);
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "follower");
    string obstacleCloudTopic;

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    nhp.param("MIN_FRONT_VISION", MIN_FRONT_VISION,
              0.3f);  // At least to the front of the car
    nhp.param("MAX_FRONT_VISION", MAX_FRONT_VISION, 5.0f);
    nhp.param("GOAL_DIST", GOAL_DIST, 2.0f);
    nhp.param("GOAL_MARGIN_OF_ERR", GOAL_MARGIN_OF_ERR, 0.2f);  // Positive
    nhp.param("MIN_SIDE_VISION", MIN_SIDE_VISION, -.2f);
    nhp.param("MAX_SIDE_VISION", MAX_SIDE_VISION, .2f);
    nhp.param("FOLLOWER_SPEED", FOLLOWER_SPEED, 1.0f);
    nhp.param("INPUT_CLOUD_TOPIC", obstacleCloudTopic, string("/map"));

    nhp.param("topic_from_plant", topic_from_plant, string("/state"));
    nhp.param("setpoint_topic", setpoint_topic, string("/setpoint"));
    nhp.param("topic_from_controller", topic_from_controller, string("/control_effort"));


    pid_speed_pub = nh.advertise<std_msgs::Float64>(topic_from_plant, 1);
    pid_setpoint_pub = nh.advertise<std_msgs::Float64>(setpoint_topic, 1);

    speed_pub = nh.advertise<rr_msgs::speed>("/speed", 1);

    auto pid_sub = nh.subscribe(topic_from_controller, 1, pidCallback);

    auto map_sub = nh.subscribe(obstacleCloudTopic, 1, mapCallback);
//    speed_pub = nh.advertise<rr_msgs::speed>("speed", 1);
    steer_pub = nh.advertise<rr_msgs::steering>("/steering", 1);



    ROS_INFO("follower initialized");

    ros::spin();
    return 0;
}
