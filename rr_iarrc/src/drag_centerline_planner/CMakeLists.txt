find_package( OpenCV REQUIRED ) # locate OpenCV in system
include_directories( ${OpenCV_INCLUDE_DIRS} ) # provide library headers

add_library(PID PID.h PID.cpp)

add_executable(drag_centerline_planner drag_centerline_planner.cpp)
target_link_libraries(drag_centerline_planner PID ${catkin_LIBRARIES})
target_link_libraries( drag_centerline_planner ${OpenCV_LIBS} )
add_dependencies(drag_centerline_planner rr_platform_generate_messages_cpp)

add_executable(drag_centerline_planner_contours drag_centerline_planner_contours.cpp)
target_link_libraries(drag_centerline_planner_contours PID ${catkin_LIBRARIES})
target_link_libraries( drag_centerline_planner_contours ${OpenCV_LIBS} )
add_dependencies(drag_centerline_planner_contours rr_platform_generate_messages_cpp)
