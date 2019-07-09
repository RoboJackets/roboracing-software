find_package( OpenCV REQUIRED ) # locate OpenCV in system
include_directories( ${OpenCV_INCLUDE_DIRS} ) # provide library headers

add_library(PID PID.h PID.cpp)

add_executable(drag_centerline_planner drag_centerline_planner.cpp)
target_link_libraries(drag_centerline_planner PID ${catkin_LIBRARIES})
target_link_libraries( drag_centerline_planner ${OpenCV_LIBS} )
add_dependencies(drag_centerline_planner rr_platform_generate_messages_cpp)

add_executable(drag_centerline_planner_daniel drag_centerline_planner_daniel.cpp)
target_link_libraries(drag_centerline_planner_daniel PID ${catkin_LIBRARIES})
target_link_libraries( drag_centerline_planner_daniel ${OpenCV_LIBS} )
add_dependencies(drag_centerline_planner_daniel rr_platform_generate_messages_cpp)
