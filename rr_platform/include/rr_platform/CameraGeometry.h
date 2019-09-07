#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <ros/node_handle.h>
#include <sensor_msgs/CameraInfo.h>

namespace rr {

class CameraGeometry {
public:
  CameraGeometry() = default;

  /**
   * Load the camera pose, field of view, and other information. Note that
   * this requires camera_info and tf to be published in order to return.
   * @param nh Reference to the node handle
   * @param camera_info_topic Topic for camera_info, e.g. /camera/camera_info
   * @param camera_link_name URDF/tf link name for the camera (x forward). E.g. "camera"
   * @param timeout Time after which this should give up
   * @return Whether the information was loaded within "timeout" seconds. If false,
   *         user must publish information on the appropriate topics or make up values.
   */
  bool LoadInfo(ros::NodeHandle& nh, const std::string& camera_info_topic, const std::string& camera_link_name,
                double timeout);

  /**
   * Return loaded camera pose, both location and quaternion orientation.
   * @return Full camera pose as a geometry_msgs::Pose
   */
  geometry_msgs::Pose GetCameraPose();

  /**
   * Return loaded camera position. Forward distance from base_footprint to camera is
   * GetCameraLocation().x
   * @return Location as geometry_msgs::Point
   */
  geometry_msgs::Point GetCameraLocation();

  /**
   * Return loaded camera orientation as a quaternion.
   * @return geometry_msgs::Quaternion
   */
  geometry_msgs::Quaternion GetCameraOrientationQuat();

  /**
   * Return the loaded camera orientation as a (roll, pitch, yaw) triple. Positive
   * pitch is down, and positive yaw is counter-clockwise from overhead.
   * @return roll, pitch, yaw
   */
  std::tuple<double, double, double> GetCameraOrientationRPY();

  /**
   * Return horizontal field of view in radians
   * @return Angular distance between leftmost and rightmost pixels
   */
  double GetFOVHorizontal();

  /**
   * Return vertical field of view in radians
   * @return Angular distance between topmost and bottommost pixels
   */
  double GetFOVVertical();

  /**
   * @return the height of the image from camera_info
   */
  int GetImageHeight();

  /**
   * @return the width of the image from camera_info
   */
  int GetImageWidth();

  /**
   * Get angular distance from center row of image to another. For the bottommost
   * row, this is GetFOVVertical() / 2.
   * @param row Image pixel row in question
   * @return Angle from center
   */
  double AngleFromCenterRow(int row);

  /**
   * Get angular distance from center column of image to another. For the leftmost
   * column, this is GetFOVHorizontal() / 2.
   * @param col Image pixel column in question
   * @return Angle from center
   */
  double AngleFromCenterColumn(int col);

  /**
   * Project the specified pixel location into world-space. Imagine drawing a line
   * from the focal point, through the image plane, and onto the ground plane.
   * @param row Pixel y location
   * @param col Pixel x location
   * @return 0: bool, true if pixel is below the horizon.
   *         1: geometry_msgs::Point for world-space point. x < 10m, -10 < y < 10,
   *            and z = 0.
   */
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
