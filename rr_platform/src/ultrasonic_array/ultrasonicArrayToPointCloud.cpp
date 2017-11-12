#include <ros/ros.h>
#include <ros/publisher.h>
#include <tf/transform_listener.h>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp> //#TODO: is this the right import?

using namespace std;

#define NUM_SENSORS 8
#define RADIUS 0.15 //radius in m




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

vector<float> parseLine(string line) {
  vector<float> dist;
  vector<string> strs;
  boost::split(strs, line, boost::is_any_of(","));

  for (int i = 0; i < strs.size(); i++) {
    dist[i] = stof(strs[i]); //convert string to float
  }

  return dist;
}

void drawSemiCircle(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointXYZ point, float radius, int numPoints) {
  numPoints -= 1; //aleady have one point
  float angle = 180.0 / (numPoints);
  float center = point.x + radius;


  float currentAngle = 360.0;

  for (int i = 0; i < numPoints; i++) {
    int x = radius * (cos(currentAngle)) + point.x;
    int y = radius * (sin(currentAngle)) + point.y;
    pcl::PointXYZ newPoint = new PointXYZ(x, y, 0.0);
    cloud += newPoint;
    //#TODO: finish this because it currently is not thought out to to ensure that the point given is always in right spot
    currentAngle -= angle;
  }

}




ros::Publisher pub;


int main(int argc, char** argv) {
	ros::init(argc, argv, "ultrasonic_array");

	ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

	pub = nh.advertise<sensor_msgs::PointCloud2>("/ultrasonic_array", 1);

// Serial port setup
    string serial_port_name;
    nhp.param(string("serial_port"), serial_port_name, string("/dev/ttyACM0")); //#TODO: launch file
    nhp.param(string("sensor_base_link"), sensor_base_link, string("ultrasonic_array_base"));
    nhp.param(string("sensor_link"), sensor_link, string("ultrasonic"));
    nhp.param(string("rate"), rate, 10); //#TODO
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
    tf::Transform tf_transform;
    pcl::PointCloud<pcl::PointXYZ> compiled_cloud;

    //######


//#######
  while(ros::ok() && serial.is_open()) {
    ros::spinOnce();

    string line = readLine();
    vector<float> distances = parseLine(line);

    for (int i = 0; i < NUM_SENSORS; i++) {
      pcl::PointXYZ point = new PointXYZ(distances[i], 0.0, 0.0);
      clouds[i] += point; //add point given
      drawSemiCircle()//#TODO: this method and propely calling it


      if(tf_listener.waitForTransform(sensor_base_link, sensor_link + to_string(i), ros::Time(0), ros::Duration(3.0))) {
        tf_listener.lookupTransform(sensor_base_link, sensor_link + to_string(i), ros::Time(0), transform);
        //transform cloud to base_link frame
        //pcl::transformPointCloud(base_link, cloud[i], cloud[i], listener); #TODO: can we use this and get rid of the listener if above?? I don't understand the co
        pcl::transformPointCloud(cloud[i], cloud[i], transform);
      }

      //add point clouds to the output one
      pcl::concatenatePointCloud(cloud[i], compiled_cloud, compiled_cloud);
    }


    //##########################
    sensor_msgs::PointCloud outmsg;
    outmsg.header = sensor_base_link;
    outmsg.points = compiled_cloud; //#TODO: um I don't think this is the proper way

    pub.publish(outmsg);

    rate.sleep();
  }






  serial.close();
  return 0;
}
