add_executable(transform_tester transform_tester.cpp)
target_link_libraries(transform_tester ${catkin_LIBRARIES})

add_executable(calibration_tester calibration_tester.cpp)
target_link_libraries(calibration_tester ${catkin_LIBRARIES})
