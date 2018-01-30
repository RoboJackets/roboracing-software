#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp> //#TODO: is this the right import?

using namespace std;

#define NUM_SENSORS 3
#define RADIUS 0.4 //radius in m
#define NUM_POINTS 12 //# points for each semicircle/circle/wall
#define PI 3.1415926535897f //#TODO: is there a better way?

#define BAUD_RATE 9600 //rate of transfer. Needs to match Arduino




string readLine(boost::asio::serial_port &port) { //#TODO: ensure this reads the lines right and doesn't include newline
    string line = "";
    bool inLine = false;
    while (true) {
        char in;
        try {
            boost::asio::read(port, boost::asio::buffer(&in, 1));
        } catch (
                boost::exception_detail::clone_impl <boost::exception_detail::error_info_injector<boost::system::system_error>> &err) {
            ROS_ERROR("Error reading serial port.");
            ROS_ERROR_STREAM(err.what());
            return line;
        }
        if (in == '\n') {
            return line;
        }
        line += in;
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
  float angleStep = (PI / 2.0) / numPoints;

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
  double angleStep = (2.0 * PI) / numPoints;

  double angle = 0; //start angle

  for (int i = 0; i < numPoints; i++) {
    //draw a circler
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
int baud_rate = 9600;

int main(int argc, char** argv) {
	ros::init(argc, argv, "ultrasonic_array");

	ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

	pub = nh.advertise<sensor_msgs::PointCloud2>("/ultrasonic_array", 1);

// Serial port setup
    string serial_port_name;
//    nhp.param(string("serial_port_name"), serial_port_name, string("/dev/ttyUSB0")); //#TODO: launch file
//    nhp.param(string("sensor_base_link"), sensor_base_link, string("ultrasonic_array_base"));
//    nhp.param(string("sensor_link"), sensor_link, string("ultrasonic_"));
//    nhp.param(string("rate"), rate_time, 10.0f); //#TODO
    serial_port_name = string("/dev/ttyUSB0");
    sensor_base_link = string("ultrasonic_array_base");
    sensor_link = string("ultrasonic_");
    rate_time = 10.0f; //#TODO want from launch file (and serial port and base link)

    boost::asio::io_service io_service;
    boost::asio::serial_port serial(io_service, serial_port_name);
   serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));

    // wait for microcontroller to start
    ros::Duration(2.0).sleep(); //#TODO: taken from motor_relay_node may not need this
    ros::Rate rate(rate_time);

//#########################
    //#TODO: Get serial port and sensor_base_link from launch file
    //#TODO: make urdf have the link of all the sensors and have a num 0 = left higher = more right

  //#####
    tf::TransformListener tf_listener;
    tf::StampedTransform tf_transform;
    pcl::PointCloud<pcl::PointXYZ> compiled_cloud;

    //######



//#######
  while(ros::ok() && serial.is_open()) {
    ros::spinOnce();

    string line = readLine(serial);
    ROS_INFO_STREAM(line);
    vector<double> distances = parseLine(line);



    for (int i = 0; i < NUM_SENSORS; i++) {

      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::PointXYZ point(distances[i], 0.0, 0.0);

      cloud.push_back(point); //add point direct from Arduino sensor
      drawSemiCircle(cloud, point, RADIUS, NUM_POINTS);
      //drawWall(cloud, point, 0.4, 4); //#TODO:is a wall better than semi circle?
      //drawCircle(cloud, point, 0.4, 6);

      if(tf_listener.waitForTransform(sensor_base_link, sensor_link + to_string(i), ros::Time(0), ros::Duration(1.0))) {
        tf_listener.lookupTransform(sensor_base_link, sensor_link + to_string(i), ros::Time(0), tf_transform);
        //transform cloud to base_link frame
        //pcl::transformPointCloud(base_link, cloud[i], cloud[i], listener); #TODO: can we use this and get rid of the listener if above?? I don't understand the co
        pcl_ros::transformPointCloud(cloud, cloud, tf_transform);
      }


      //add point cloud to the output one
      compiled_cloud += cloud;
    }


    //##########################
    sensor_msgs::PointCloud2 outmsg;
    pcl::toROSMsg(compiled_cloud, outmsg);
    outmsg.header.frame_id = sensor_base_link;//"base_footprint";//sensor_base_link; #TODO SET AS SENSOR BASE LINK and should we set the compiled cloud header instead?


    pub.publish(outmsg);
    compiled_cloud.clear(); //remove old data for next round
    rate.sleep();
  }


  ROS_INFO_STREAM("Shutting down Ultrasonic Array");
  serial.close();

  return 0;
}
