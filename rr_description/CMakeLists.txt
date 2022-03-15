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
        # urdf/rigatoni.urdf.xacro
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


ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/rr_description.dsv.in")

ament_package()
