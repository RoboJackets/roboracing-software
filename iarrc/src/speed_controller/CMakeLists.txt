add_executable(speed_controller speed_controller.cpp)
target_link_libraries(speed_controller ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
