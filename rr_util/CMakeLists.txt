cmake_minimum_required(VERSION 3.8)
project(rr_util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(rr_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


add_library(${PROJECT_NAME}_components src/image_flipper/image_flipper.cpp)

add_subdirectory(src/planner)
add_subdirectory(src/image_pcl_converter)
add_subdirectory(src/pointcloud_combiner)
add_subdirectory(src/navigation_controller)
add_subdirectory(src/follower)
add_subdirectory(src/rosbag_remote_recorder)
add_subdirectory(src/local_mapper)
add_subdirectory(src/pose_tracker)
add_subdirectory(src/camera_geometry)
add_subdirectory(src/image_transformation)
add_subdirectory(src/color_filter)

ament_target_dependencies(${PROJECT_NAME}_components
"rclcpp"
"rclcpp_components"
"cv_bridge"
"pcl_ros"
"image_transport"
"rr_msgs"
"nav2_msgs"
"tf2_ros"
)

###########
## Build ##
###########

install(TARGETS ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)