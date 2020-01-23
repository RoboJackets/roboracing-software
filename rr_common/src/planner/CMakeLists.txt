add_library(nearest_point_cache nearest_point_cache.cpp)
target_link_libraries(nearest_point_cache ${catkin_LIBRARIES})

add_library(inflation_map inflation_map.cpp)
target_link_libraries(inflation_map ${catkin_LIBRARIES})

add_library(distance_map distance_map.cpp)
target_link_libraries(distance_map ${catkin_LIBRARIES})

add_library(bicycle_model bicycle_model.cpp)
target_link_libraries(nearest_point_cache ${catkin_LIBRARIES})

set(planning_libs nearest_point_cache inflation_map distance_map bicycle_model)

add_library(annealing_optimizer annealing_optimizer.cpp)
target_link_libraries(annealing_optimizer ${catkin_LIBRARIES})

add_library(hill_climb_optimizer hill_climb_optimizer.cpp)
target_link_libraries(hill_climb_optimizer ${catkin_LIBRARIES})

add_executable(planner planner_node.cpp)
target_link_libraries(planner
        bicycle_model
        nearest_point_cache
        inflation_map
        distance_map
        annealing_optimizer
        hill_climb_optimizer
        ${catkin_LIBRARIES})
add_dependencies(planner ${catkin_EXPORTED_TARGETS})
