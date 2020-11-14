cmake_minimum_required(VERSION 3.0.2)
project(rr_description)

## Find catkin macros and libraries
find_package(catkin
        REQUIRED COMPONENTS
        xacro
)

catkin_package(
#        INCLUDE_DIRS include
#        LIBRARIES ${PROJECT_NAME}
#        CATKIN_DEPENDS xacro
#        DEPENDS eigen opencv
)

file(GLOB xacro_files urdf/evgp.urdf.xacro urdf/evgp_opponent.urdf.xacro urdf/rigatoni.urdf.xacro)
xacro_add_files(${xacro_files} TARGET xacro_files)
