#include "rr_common/CameraGeometry.h"

#include <cmath>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "rr_common/angle_utils.hpp"


namespace rr {

void CameraGeometry::CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
  image_size_rows_ = msg->height;
  image_size_cols_ = msg->width;
  double fx = msg->P[0];  // horizontal focal length of rectified image, in px
  double fy = msg->P[5];  // vertical focal length
  cam_fov_x_ = 2 * atan2(image_size_cols_, 2*fx);
  cam_fov_y_ = 2 * atan2(image_size_rows_, 2*fy);
  received_camera_info_ = true;
}

bool CameraGeometry::LoadFOV(ros::NodeHandle &nh, const std::string &camera_info_topic,
                             const double timeout)
{
  received_camera_info_ = false;

  // temporary camera info subscriber is destroyed after one message
  auto cam_info_sub = nh.subscribe(camera_info_topic, 1, &CameraGeometry::CameraInfoCallback, this);

  ros::Time t_start = ros::Time::now();
  while (!received_camera_info_) {
    if ((ros::Time::now() - t_start).toSec() > timeout) {
      ROS_WARN("[CameraGeometry] setting camera FOV from camera_info timed out");
      break;
    }

    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  return received_camera_info_;
}

bool CameraGeometry::LoadCameraPose(const std::string &camera_link_name, const double timeout) {
  geometry_msgs::PoseStamped ps_src_cam;  // source pose, origin
  geometry_msgs::PoseStamped ps_dst_base;  // destination pose, camera frame origin from base_footprint

  ps_src_cam.pose.orientation.w = 1.0;  // set orientation in x-axis direction
  ps_src_cam.header.frame_id = camera_link_name;

  tf::TransformListener listener;
  bool success = false;
  while (!success) {
    try {
      listener.waitForTransform("base_footprint", camera_link_name, ros::Time(0), ros::Duration(timeout));
      listener.transformPose("base_footprint", ps_src_cam, ps_dst_base);
      success = true;
    } catch (tf::LookupException& e) {
      ROS_ERROR("tf lookup exception in LoadCameraPose: %s", e.what());
    }
  }

  camera_pose_ = ps_dst_base.pose;

  return success;
}

bool CameraGeometry::LoadInfo(ros::NodeHandle& nh, const std::string& camera_info_topic,
                              const std::string& camera_link_name, const double timeout)
{
  bool success = true;
  success &= LoadFOV(nh, camera_info_topic, timeout);
  success &= LoadCameraPose(camera_link_name, timeout);
  return success;
}

geometry_msgs::Pose CameraGeometry::GetCameraPose() {
  return camera_pose_;
}

geometry_msgs::Point CameraGeometry::GetCameraLocation() {
  return camera_pose_.position;
}

geometry_msgs::Quaternion CameraGeometry::GetCameraOrientationQuat() {
  return camera_pose_.orientation;
}

std::tuple<double, double, double> CameraGeometry::GetCameraOrientationRPY() {
  return rr::poseToRPY(camera_pose_);
}

double CameraGeometry::GetFOVHorizontal() {
  return cam_fov_x_;
}

double CameraGeometry::GetFOVVertical() {
  return cam_fov_y_;
}

int CameraGeometry::GetImageHeight() {
  return image_size_rows_;
}

int CameraGeometry::GetImageWidth() {
  return image_size_cols_;
}

double CameraGeometry::AngleFromCenterRow(int row) {
  const auto center_row = image_size_rows_ * 0.5;
  const auto row_diff = center_row - row;  // up (low row) is positive difference to match pitch
  const auto angle_per_row = cam_fov_y_ / image_size_rows_;
  return row_diff * angle_per_row;
}

double CameraGeometry::AngleFromCenterColumn(int col) {
  const auto center_col = image_size_cols_ * 0.5;
  const auto col_diff = center_col - col;  // left (low col) is positive difference to match yaw
  const auto angle_per_col = cam_fov_x_ / image_size_cols_;
  return col_diff * angle_per_col;
}

std::tuple<bool, geometry_msgs::Point> CameraGeometry::ProjectToWorld(int row, int col) {
  const auto [cam_roll, cam_pitch, cam_yaw] = GetCameraOrientationRPY();

  // pitch is negative if pointing down to ground, so invert it
  const auto pitch_ground_relative = -(cam_pitch + AngleFromCenterRow(row));
  const auto yaw_ground_relative = AngleFromCenterColumn(col);

  // first compute the x and y location relative to the camera
  tf::Vector3 point_relative;
  point_relative.setX(camera_pose_.position.z / std::tan(pitch_ground_relative));
  point_relative.setY(point_relative.x() * std::tan(yaw_ground_relative));
  point_relative.setZ(0);

  // rotate to match camera pose
  tf::Vector3 point_rotated = tf::quatRotate(tf::createQuaternionFromYaw(cam_yaw), point_relative);

  geometry_msgs::Point point_world;
  point_world.x = point_rotated.x() + camera_pose_.position.x;
  point_world.y = point_rotated.y() + camera_pose_.position.y;
  point_world.z = 0;

  return std::make_tuple(pitch_ground_relative > 0, point_world);
}

}  // namespace rr
