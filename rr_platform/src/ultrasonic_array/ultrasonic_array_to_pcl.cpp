#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp> //#TODO: is this the right import?

using namespace std;

#define NUM_SENSORS 8
#define RADIUS 0.15 //radius in m
#define NUM_POINTS 5 //# points for each semicircle
#define PI 3.1415926535897 //#TODO: is there a better way?




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
    dist[i] = atof(strs[i].c_str()); //convert string to double
  }

  return dist;
}

void drawSemiCircle(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointXYZ point, float radius, int numPoints) {
  numPoints = numPoints - 1; //already have 1 point plotted
  int numAngleShifts =  numPoints - 1; //numPoints - 1 = # of angleShifta
  float angle = (2 * PI ) / (numAngleShifts);
  float center = point.x + radius;


  float currentAngle = 3 * PI / 2; //#TODO: shift if necssary? Should provide semicircle with outside facing the robot

  for (int i = 0; i < numPoints; i++) {
    float x = radius * (cos(currentAngle)) + point.x;
    float y = -radius * (sin(currentAngle)) + point.y; //- because urdf standards of left is +y
    pcl::PointXYZ newPoint(x, y, 0.0);
    cloud.push_back(newPoint);
    currentAngle -= angle;
  }

}




ros::Publisher pub;
string sensor_base_link;
string sensor_link;
float rate;
int baud_rate;

int main(int argc, char** argv) {
	ros::init(argc, argv, "ultrasonic_array");

	ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

	pub = nh.advertise<sensor_msgs::PointCloud2>("/ultrasonic_array", 1);

// Serial port setup
    string serial_port_name;
    nhp.param(string("serial_port_name"), serial_port_name, string("/dev/ttyACM0")); //#TODO: launch file
    nhp.param(string("sensor_base_link"), sensor_base_link, string("ultrasonic_array_base"));
    nhp.param(string("sensor_link"), sensor_link, string("ultrasonic_"));
    nhp.param(string("rate"), rate, 10.0f); //#TODO
    boost::asio::io_service io_service;
    boost::asio::serial_port serial(io_service, serial_port_name);
    serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));

    // wait for microcontroller to start
    ros::Duration(2.0).sleep(); //#TODO: taken from motor_relay_node may not need this
    ros::Rate rate(rate);//#TODO: rate from arduino?

//#########################
    //#TODO: Get serial port and sensor_base_link from launch file
    //#TODO: make urdf have the link of all the sensors and have a num 0 = left higher = more right

  //#####
    vector<pcl::PointCloud<pcl::PointXYZ>> clouds;
    tf::TransformListener tf_listener;
    tf::StampedTransform tf_transform;
    pcl::PointCloud<pcl::PointXYZ> compiled_cloud;

    //######


//#######
  while(ros::ok() && serial.is_open()) {
    ros::spinOnce();

    string line = readLine(serial);
    vector<double> distances = parseLine(line);

    for (int i = 0; i < NUM_SENSORS; i++) {
      pcl::PointXYZ point(distances[i], 0.0, 0.0);
      clouds[i].push_back(point); //add point given
      drawSemiCircle(clouds[i], point, RADIUS, NUM_POINTS); //numPoints is number of points including the one we added line above!


      if(tf_listener.waitForTransform(sensor_base_link, sensor_link + to_string(i), ros::Time(0), ros::Duration(3.0))) {
        tf_listener.lookupTransform(sensor_base_link, sensor_link + to_string(i), ros::Time(0), tf_transform);
        //transform cloud to base_link frame
        //pcl::transformPointCloud(base_link, cloud[i], cloud[i], listener); #TODO: can we use this and get rid of the listener if above?? I don't understand the co
        pcl_ros::transformPointCloud(clouds[i], clouds[i], tf_transform);
      }

      //add point clouds to the output one
      compiled_cloud += clouds[i];
      //pcl::concatenatePointCloud(clouds[i], compiled_cloud, compiled_cloud);
    }


    //##########################
    sensor_msgs::PointCloud2 outmsg;
    outmsg.header.frame_id = sensor_base_link;
    pcl::toROSMsg(compiled_cloud, outmsg);

    pub.publish(outmsg);

    rate.sleep();
  }






  serial.close();
  return 0;
}
