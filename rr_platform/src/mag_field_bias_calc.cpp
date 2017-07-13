#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>

using namespace std;

double x_max = -numeric_limits<double>::infinity();
double x_min = numeric_limits<double>::infinity();
double y_max = -numeric_limits<double>::infinity();
double y_min = numeric_limits<double>::infinity();
double z_max = -numeric_limits<double>::infinity();
double z_min = numeric_limits<double>::infinity();

double x_bias = 0;
double y_bias = 0;
double z_bias = 0;


void callback(const sensor_msgs::MagneticFieldConstPtr &msg) {
  x_max = max(x_max, msg->magnetic_field.x);
  x_min = min(x_min, msg->magnetic_field.x);
  y_max = max(y_max, msg->magnetic_field.y);
  y_min = min(y_min, msg->magnetic_field.y);
  z_max = max(z_max, msg->magnetic_field.z);
  z_min = min(z_min, msg->magnetic_field.z);

  x_bias = (x_min + x_max) / 2.0;
  y_bias = (y_min + y_max) / 2.0;
  z_bias = (z_min + z_max) / 2.0;

  ROS_INFO_STREAM("x: " << x_bias << "\ty: " << y_bias << "\tz: " << z_bias);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "magnetic_field_bias_calc");

  ros::NodeHandle nh;

  auto subscriber = nh.subscribe("/imu/mag", 1, callback);

  ros::spin();

  return 0;
}
