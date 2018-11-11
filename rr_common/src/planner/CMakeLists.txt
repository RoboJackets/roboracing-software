add_executable(planner planner_node.cpp planner.cpp planner.h)
target_link_libraries(planner ${catkin_LIBRARIES})
add_dependencies(planner rr_platform_generate_messages_cpp)

catkin_install_python(
        PROGRAMS planner_plotter.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
