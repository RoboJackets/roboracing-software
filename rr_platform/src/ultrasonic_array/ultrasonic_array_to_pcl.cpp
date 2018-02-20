#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/math/constants/constants.hpp>
#include <algorithm>
#include <string>

using namespace std;

/*
  README
  @author Brian Cochran, github: @btdubs
  Node for taking in serial CSV distances from ultrasonic array,
  then outputing a point cloud. Ultrasonic sensors defined in
  urdf and uses that for transforming points.
  Requires a tf tree being published.
*/


//defaults for launch file paramenters
#define NUM_SENSORS_DEFAULT 6
#define DRAW_METHOD_DEFAULT 0 //0 = Single Point, 1 = wall, 2 = circle, 3 = semicircle
#define NUM_POINTS_DEFAULT 12 //# minimum points for each semicircle/circle/wall
#define RADIUS_DEFAULT 0.1f //radius of semicircle/circle in meters
#define LENGTH_DEFAULT 0.1f //length of wall
#define DISTANCE_CLIP_DEFAULT 5.0f //distance beyond which we ignore point in meters
#define BAUD_RATE_DEFAULT 9600 //rate of transfer. Needs to match Arduino

const double pi = boost::math::constants::pi<double>();


string readLine(boost::asio::serial_port &port) {
    string line = "";
    bool inLine = false;
    while (true) {
        char in;
        try {
            boost::asio::read(port, boost::asio::buffer(&in, 1));
            if (in == '\n') {
                return line;
            }
            line += in;
        } catch (
            boost::exception_detail::clone_impl <boost::exception_detail::error_info_injector<boost::system::system_error>> &err) {
            ROS_ERROR("Error reading serial port.");
            ROS_ERROR_STREAM(err.what());
        }

    }
}

vector<double> parseLine(string line) {
  vector<string> strs;
  boost::split(strs, line, boost::is_any_of(","));

  vector<double> dist(strs.size());
  std::transform(strs.begin(), strs.end(), dist.begin(), [](const auto &s){return std::stod(s);}); //convert string to double

  return dist;
}

void drawWall(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointXYZ center, float length, int numPoints) {
  length = (length + 1) / 2.0; //@Note: adds one to ensure we get at least the numPoints desired
  float step = length / (float) numPoints / 2.0;
  float dist = step;
  for (int i = 0; i < numPoints / 2; i++) {
    //draw half the line and mirror
    pcl::PointXYZ point1(center.x, dist, 0); //#TODO: might need to swap x and y
    pcl::PointXYZ point2(center.x, -dist, 0); //#TODO: might need to swap x and y

    cloud.push_back(point1);
    cloud.push_back(point2);
    dist = dist + step;
  }

}

void drawSemiCircle(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointXYZ point, float radius, int numPoints) {
  numPoints = (numPoints + 1) / 2; //@Note: adds one to ensure we get at least the numPoints desired
  float angleStep = (pi / 2.0) / numPoints;

  float angle = 0.0; //#TODO: may need to shift this start angle

  for (int i = 0; i < numPoints ; i++) {
    //draw a quarter of a cicle and mirror
    float x = radius * cos(angle);
    float y = radius * sin(angle);

    cloud.push_back(pcl::PointXYZ(point.x + radius - x, y, 0));
    cloud.push_back(pcl::PointXYZ(point.x + radius - x, -y, 0));

    angle = angle + angleStep;
  }

}

void drawCircle(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointXYZ point, double radius, int numPoints) {
  double angleStep = (2.0 * pi) / numPoints;

  double angle = 0; //start angle

  for (int i = 0; i < numPoints; i++) {
    //draw a circle
    double x = radius * cos(angle);
    double y = radius * sin(angle);

    cloud.push_back(pcl::PointXYZ(point.x + radius + x, y, 0)); //point.x + radius is center of circle
    angle = angle + angleStep;

  }

}



int main(int argc, char** argv) {

  ros::init(argc, argv, "ultrasonic_array");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/ultrasonic_array", 1);

  //Roslaunch file
  string sensor_base_link;
  string sensor_link;
  float update_rate;
  string serial_port_name;
  float distance_clip;
  float length;
  float radius;
  int num_sensors;
  int num_points;
  int draw_method;
  int baud_rate;
  nhp.param(string("serial_port_name"), serial_port_name, string("/dev/ttyUSB0"));
  nhp.param(string("sensor_base_link"), sensor_base_link, string("ultrasonic_array_base"));
  nhp.param(string("sensor_link_prefix"), sensor_link, string("ultrasonic_"));
  nhp.param(string("update_rate"), update_rate, 10.0f);
  nhp.param(string("number_of_sensors"), num_sensors, NUM_SENSORS_DEFAULT);
  nhp.param(string("baud_rate"), baud_rate, BAUD_RATE_DEFAULT);
  nhp.param(string("distance_clip"), distance_clip, DISTANCE_CLIP_DEFAULT);
  nhp.param(string("draw_method"), draw_method, DRAW_METHOD_DEFAULT);
  nhp.param(string("wall_length"), length, LENGTH_DEFAULT);
  nhp.param(string("circle_radius"), radius, RADIUS_DEFAULT);
  nhp.param(string("number_of_points"), num_points, NUM_POINTS_DEFAULT);

  //Connect serial
  ROS_INFO_STREAM("Connecting to serial at port: " + serial_port_name);
  boost::asio::io_service io_service;
  boost::asio::serial_port serial(io_service, serial_port_name);
  serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));

  // wait for microcontroller to start
  ros::Duration(2.0).sleep();

  ros::Rate rate(update_rate);

  //Transforms
  tf::TransformListener tf_listener;
  tf::StampedTransform tf_transform;
  pcl::PointCloud<pcl::PointXYZ> compiled_cloud;

  //lookup transforms and store them as they do not change
  vector<tf::StampedTransform> tf_transform_vector;
  for (int i = 0; i < num_sensors; i++) {
    if(tf_listener.waitForTransform(sensor_base_link, sensor_link + to_string(i), ros::Time(0), ros::Duration(5.0))) {
      tf_listener.lookupTransform(sensor_base_link, sensor_link + to_string(i), ros::Time(0), tf_transform);
    }

    tf_transform_vector.push_back(tf_transform);
  }

  function<void(pcl::PointCloud<pcl::PointXYZ> &a, pcl::PointXYZ &b)> draw;
  switch (draw_method) {
      default:
      case 0: draw = [](pcl::PointCloud<pcl::PointXYZ> a, pcl::PointXYZ b){};
              break;
      case 1: draw = bind(drawWall, _1, _2, length, num_points);
              break;
      case 2: draw = bind(drawCircle, _1, _2, radius, num_points);
              break;
      case 3: draw = bind(drawSemiCircle, _1, _2, radius, num_points);
              break;
  }

  while(ros::ok() && serial.is_open()) {
    ros::spinOnce();

    string line = readLine(serial);
    ROS_INFO_STREAM(line);
    vector<double> distances = parseLine(line);



    for (int i = 0; i < num_sensors; i++) {
      pcl::PointCloud<pcl::PointXYZ> cloud;

      if(distances[i] < distance_clip) {
        pcl::PointXYZ point(distances[i], 0.0, 0.0);

        cloud.push_back(point); //add point direct from Arduino sensor
        draw(cloud, point);

        pcl_ros::transformPointCloud(cloud, cloud, tf_transform_vector[i]);

        //add point cloud to the output cloud
        compiled_cloud += cloud;
      }
    }

    sensor_msgs::PointCloud2 outmsg;
    pcl::toROSMsg(compiled_cloud, outmsg);
    outmsg.header.frame_id = sensor_base_link;


    pub.publish(outmsg);
    compiled_cloud.clear(); //remove old data for next round
    rate.sleep();
  }


  ROS_INFO_STREAM("Shutting down Ultrasonic Array");
  serial.close();

  return 0;
}
