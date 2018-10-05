set(planner_hdrs distance_checker.h bicycle_model.h planner.h planner_types.h
                 random_sample_planner.h annealing_planner.h)

add_library(distance_checker distance_checker.cpp ${planner_hdrs})
target_link_libraries(distance_checker ${catkin_LIBRARIES})

add_library(kinematic_bicycle_model bicycle_model.cpp ${planner_hdrs})
target_link_libraries(kinematic_bicycle_model ${catkin_LIBRARIES})

add_library(random_sample_planner random_sample_planner.cpp ${planner_hdrs})
target_link_libraries(random_sample_planner
        kinematic_bicycle_model distance_checker ${catkin_LIBRARIES}
)

add_library(annealing_planner annealing_planner.cpp ${planner_hdrs})
target_link_libraries(annealing_planner
        kinematic_bicycle_model distance_checker ${catkin_LIBRARIES}
)

add_executable(planner planner_node.cpp)
target_link_libraries(planner random_sample_planner annealing_planner ${catkin_LIBRARIES})
add_dependencies(planner rr_platform_generate_messages_cpp)
