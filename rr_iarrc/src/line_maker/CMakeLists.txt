find_package( OpenCV REQUIRED ) # locate OpenCV in system
include_directories( ${OpenCV_INCLUDE_DIRS} ) # provide library headers

add_library(PID PID.h PID.cpp)

add_executable(line_maker line_maker.cpp)
target_link_libraries(line_maker PID ${catkin_LIBRARIES})
target_link_libraries( line_maker ${OpenCV_LIBS} )
add_dependencies(line_maker rr_platform_generate_messages_cpp)
