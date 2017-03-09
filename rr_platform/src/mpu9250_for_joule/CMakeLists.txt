if(${mraa_FOUND})
add_executable(mpu9250_for_joule main.cpp)
target_link_libraries(mpu9250_for_joule ${catkin_LIBRARIES} mraa)
endif(${mraa_FOUND})