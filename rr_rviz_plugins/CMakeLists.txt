cmake_minimum_required(VERSION 3.8)
project(rr_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rr_msgs REQUIRED)

set(rviz_default_plugins_headers_to_moc
  include/Speedometer.hpp
)

set(rviz_default_plugins_source_files
  src/Speedometer.cpp
)

add_library(rr_rviz_plugins SHARED
  ${rviz_default_plugins_moc_files}
  ${rviz_default_plugins_source_files}
)

#################################################################
# target_include_directories(rviz_default_plugins PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${Qt5Widgets_INCLUDE_DIRS}
# )

# target_link_libraries(rviz_default_plugins PUBLIC
#   rviz_ogre_vendor::OgreMain
#   rviz_ogre_vendor::OgreOverlay
#   ignition-math6
# )

# target_compile_definitions(rviz_default_plugins PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

# target_compile_definitions(rviz_default_plugins PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
#################################################################

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(rr_rviz_plugins
  PUBLIC
  rclcpp
  rviz_common
  rr_msgs
  pluginlib
)

ament_export_include_directories(include)
ament_export_targets(rr_rviz_plugins HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rviz_common
  rr_msgs
  pluginlib
)

install(
  TARGETS rr_rviz_plugins
  EXPORT rr_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
