#pragma once

#include <ros/node_handle.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/CameraInfo.h>

namespace rr {

class CameraGeometry {
public:
  CameraGeometry() = default;

  bool LoadInfo(ros::NodeHandle& nh, const std::string& camera_info_topic,
                const std::string& camera_link_name, double timeout);

  geometry_msgs::Pose GetCameraPose();

  geometry_msgs::Point GetCameraLocation();

  geometry_msgs::Quaternion GetCameraOrientationQuat();

  std::tuple<double, double, double> GetCameraOrientationRPY();

  double GetFOVHorizontal();

  double GetFOVVertical();

  int GetImageHeight();

  int GetImageWidth();

  double AngleFromCenterRow(int row);

  double AngleFromCenterColumn(int col);

  std::tuple<bool, geometry_msgs::Point> ProjectToWorld(int row, int col);


protected:
  geometry_msgs::Pose camera_pose_;

  double cam_fov_x_;
  double cam_fov_y_;

  int image_size_rows_;
  int image_size_cols_;

  bool received_camera_info_;


  void CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

  bool LoadFOV(ros::NodeHandle& nh, const std::string& camera_info_topic, double timeout);

  bool LoadCameraPose(const std::string& camera_link_name, double timeout);
};

}  // namespace rr
