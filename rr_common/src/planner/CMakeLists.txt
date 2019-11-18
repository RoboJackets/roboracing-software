add_library(nearest_point_cache nearest_point_cache.cpp)

add_library(bicycle_model bicycle_model.cpp)

set(planning_libs nearest_point_cache bicycle_model)

add_library(annealing_optimizer annealing_optimizer.cpp)
target_link_libraries(annealing_optimizer ${planning_libs} ${catkin_LIBRARIES})

add_library(hill_climb_optimizer hill_climb_optimizer.cpp)
target_link_libraries(hill_climb_optimizer ${planning_libs} ${catkin_LIBRARIES})

add_executable(planner planner_node.cpp)
target_link_libraries(planner
        annealing_optimizer
        hill_climb_optimizer
        ${catkin_LIBRARIES})
add_dependencies(planner ${catkin_EXPORTED_TARGETS})

#add_executable(iarrc_planner iarrc_planner.cpp)
#target_link_libraries(iarrc_planner annealing_optimizer ${catkin_LIBRARIES})
#add_dependencies(iarrc_planner ${catkin_EXPORTED_TARGETS})

#add_executable(evgp_planner evgp_planner.cpp)
#target_link_libraries(evgp_planner hill_climb_planner ${catkin_LIBRARIES})
#add_dependencies(evgp_planner ${catkin_EXPORTED_TARGETS})
