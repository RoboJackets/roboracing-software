cmake_minimum_required(VERSION 2.8.3)
project(rr_description)

## Find catkin macros and libraries
find_package(catkin
        REQUIRED COMPONENTS
        xacro
)

catkin_package(
#        INCLUDE_DIRS include
#        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS xacro
#        DEPENDS eigen opencv
)

file(GLOB xacro_files urdf/*.urdf.xacro)
xacro_add_files(${xacro_files} TARGET xacro_files)
