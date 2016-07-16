#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transform.setRotation( tf::Quaternion(0, 1, 0, 0) );

  tf::Transform tf2;
  tf2.setOrigin(tf::Vector3(0.0, 0.0, 0.0) );
  tf2.setRotation(tf::Quaternion(0, 0, 1, 0));
  transform *= tf2;

  ros::Rate rate(15.0);
  while (node.ok()){
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "laser", "laser_inverted"));
    rate.sleep();
  }
  return 0;
};