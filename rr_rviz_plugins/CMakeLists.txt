cmake_minimum_required(VERSION 3.5)
project(rr_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rr_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

add_library(${PROJECT_NAME} SHARED
  src/Speedometer.cpp
  src/SpeedometerWorker.cpp
  src/SteeringDirection.cpp
  src/SteeringWorker.cpp
  include/Speedometer.hpp
  include/SpeedometerWorker.hpp
  include/SteeringDirection.hpp
  include/SteeringWorker.hpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)


set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_components"
  "rviz_common"
  "pluginlib"
  "rr_msgs"
  "Qt5"
)
target_compile_definitions(${PROJECT_NAME} PRIVATE "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_package()
