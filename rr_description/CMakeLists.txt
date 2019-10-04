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
#        CATKIN_DEPENDS xacro
#        DEPENDS eigen opencv
)

xacro_add_files(
        urdf/evgp.urdf.xacro
        urdf/evgp_opponent.urdf.xacro
        TARGET gen_xacro_files
)
