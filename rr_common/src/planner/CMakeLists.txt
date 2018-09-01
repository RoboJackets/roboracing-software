add_library(planner planner.cpp planner.h)
# target_

add_executable(planner-bin planner_node.cpp)
target_link_libraries(planner-bin planner ${catkin_LIBRARIES})
add_dependencies(planner-bin rr_platform_generate_messages_cpp)
set_target_properties(planner-bin PROPERTIES OUTPUT_NAME planner)

catkin_install_python(
        PROGRAMS planner_plotter.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
