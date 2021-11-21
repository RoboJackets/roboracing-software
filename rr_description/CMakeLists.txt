<<<<<<< HEAD
cmake_minimum_required(VERSION 3.8)
project(rr_description)


add_library (${PROJECT_NAME} BUILD_SHARED
    src/main.cpp
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
=======
cmake_minimum_required(VERSION 3.5)
project(rr_description)

## Find macros and libraries

include_directories(${CMAKE_SOURCE_DIR}/third_party)
find_package(xacro REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(velodyne_description REQUIRED)

xacro_add_files(
        urdf/evgp.urdf.xacro
        urdf/opponents/evgp_opponent_0.urdf.xacro
        urdf/opponents/evgp_opponent_1.urdf.xacro
        urdf/opponents/evgp_opponent_2.urdf.xacro
        urdf/opponents/evgp_opponent_3.urdf.xacro
        urdf/opponents/evgp_opponent_4.urdf.xacro
        urdf/rigatoni.urdf.xacro
	TARGET urdf_generation
	INSTALL DESTINATION urdf
)

install(
	DIRECTORY
	#launch
	urdf
	#rviz
	meshes
	media
	conf
	DESTINATION share/${PROJECT_NAME}
)
if (BUIlD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
>>>>>>> origin/ros2/main
endif()

ament_package()
