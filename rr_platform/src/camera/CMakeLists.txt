add_library(rr_camera_geometry CameraGeometry.cpp)
target_link_libraries(rr_camera_geometry ${catkin_LIBRARIES})

add_executable(image_transform image_transform.cpp)
target_link_libraries(image_transform rr_camera_geometry ${catkin_LIBRARIES})

add_library(rr_pointcloud_projector pointcloud_projector.cpp)
target_link_libraries(rr_pointcloud_projector rr_camera_geometry ${catkin_LIBRARIES})
