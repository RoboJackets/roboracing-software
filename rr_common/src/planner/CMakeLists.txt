add_library(path path_structs.cpp density_cluster.cpp)
add_executable(planner planner.cpp)
target_link_libraries(planner path ${catkin_LIBRARIES})
add_dependencies(planner rr_platform_generate_messages_cpp)

catkin_install_python(
        PROGRAMS
        planner_plotter.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
