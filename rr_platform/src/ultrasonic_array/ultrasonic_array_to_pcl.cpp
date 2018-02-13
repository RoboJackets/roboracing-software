#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/math/constants/constants.hpp>

using namespace std;

#define NUM_SENSORS 6
#define RADIUS 0.1 //radius of semicircle/circle in meters
#define NUM_POINTS 12 //# points for each semicircle/circle/wall
#define BAUD_RATE 9600 //rate of transfer. Needs to match Arduino

const double pi = boost::math::constants::pi<double>();

/*
#TODO:
      decide best point strategy
      Random todos around the place
      parameterize things not already parameterized (length of wall, which draw option)
      fix problem if serial becomes unplugged it seems to become unresponsive
*/


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
  vector<double> dist;
  vector<string> strs;
  boost::split(strs, line, boost::is_any_of(","));

  for (int i = 0; i < strs.size(); i++) {
    dist.push_back(atof(strs[i].c_str())); //convert string to double
  }

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

    angle = angle + angleStep; //#TODO: this may need to be minus because
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



ros::Publisher pub;
string sensor_base_link;
string sensor_link;
float rate_time;
string serial_port_name;
float distance_clip;

int main(int argc, char** argv) {
	ros::init(argc, argv, "ultrasonic_array");

	ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

	pub = nh.advertise<sensor_msgs::PointCloud2>("/ultrasonic_array", 1);

  //Roslaunch file
    nhp.param(string("serial_port_name"), serial_port_name, string("/dev/ttyUSB0"));
    nhp.param(string("sensor_base_link"), sensor_base_link, string("ultrasonic_array_base"));
    nhp.param(string("sensor_link"), sensor_link, string("ultrasonic_"));
    nhp.param(string("rate"), rate_time, 10.0f);
    nhp.param(string("distance_clip"), distance_clip, 5.0f);

  //Connect serial
    ROS_INFO_STREAM("Connecting to serial at port: " + serial_port_name);
    boost::asio::io_service io_service;
    boost::asio::serial_port serial(io_service, serial_port_name);
    serial.set_option(boost::asio::serial_port_base::baud_rate(BAUD_RATE));

    // wait for microcontroller to start
    ros::Duration(2.0).sleep(); //#TODO: taken from motor_relay_node may not need this

    ros::Rate rate(rate_time);

    //Transforms
    tf::TransformListener tf_listener;
    tf::StampedTransform tf_transform;
    pcl::PointCloud<pcl::PointXYZ> compiled_cloud;

    //lookup transforms and store them as they do not change
    vector<tf::StampedTransform> tf_transform_vector;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if(tf_listener.waitForTransform(sensor_base_link, sensor_link + to_string(i), ros::Time(0), ros::Duration(5.0))) {
        tf_listener.lookupTransform(sensor_base_link, sensor_link + to_string(i), ros::Time(0), tf_transform);
      }

      tf_transform_vector.push_back(tf_transform);
    }

//#TODO may want to do some sort of serial flushing here to clear out stale data in buffer

  while(ros::ok() && serial.is_open()) {
    ros::spinOnce();

    string line = readLine(serial);
    ROS_INFO_STREAM(line);
    vector<double> distances = parseLine(line);



    for (int i = 0; i < NUM_SENSORS; i++) {
      pcl::PointCloud<pcl::PointXYZ> cloud;

      if(distances[i] < distance_clip) {
        pcl::PointXYZ point(distances[i], 0.0, 0.0);

        cloud.push_back(point); //add point direct from Arduino sensor
        //drawSemiCircle(cloud, point, RADIUS, NUM_POINTS); //#TODO: SHOULD WE drawCircle after TRANSFORM TO TECHNICALLY SAVE time?
        //drawWall(cloud, point, 0.4, 4); //#TODO:is a wall better than semi circle?
        //drawCircle(cloud, point, 0.4, 6);

        pcl_ros::transformPointCloud(cloud, cloud, tf_transform_vector[i]);

        //add point cloud to the output one
        compiled_cloud += cloud;
      }
    }


    //##########################
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
