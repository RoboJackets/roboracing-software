#include <math.h>
#include <ros/ros.h>
#include <rr_platform/axes.h>
#include <sensor_msgs/Imu.h>

ros::Publisher axes_pub;
rr_platform::axes axes_msg;

float w, x, y, z;
float roll, pitch, yaw;

static void toEulerianAngle(float w, float x, float y, float z) {
  double ysqr = y * y;

  double t0 = +2.0 * (w * x + y * z);
  double t1 = +1.0 - 2.0 * (x * x + ysqr);
  roll = atan2(t0, t1);

  double t2 = +2.0 * (w * y - z * x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  pitch = asin(t2);

  double t3 = +2.0 * (w * z + x * y);
  double t4 = +1.0 - 2.0 * (ysqr + z * z);
  yaw = atan2(t3, t4);

  rr_platform::axes publishable_copy = axes_msg;
  publishable_copy.header.stamp = ros::Time::now();
  axes_pub.publish(publishable_copy);
}

void imuCB(const sensor_msgs::ImuConstPtr& msg) {
  w = msg->orientation.w;
  x = msg->orientation.x;
  y = msg->orientation.y;
  z = msg->orientation.z;

  toEulerianAngle(w, x, y, z);

  axes_msg.roll = roll;
  axes_msg.pitch = pitch;
  axes_msg.yaw = yaw;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_axes");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  axes_pub = nh.advertise<rr_platform::axes>("/axes", 1);
  auto imuSub = nh.subscribe("/imu", 1, imuCB);

  ros::Rate rate(30.0);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}