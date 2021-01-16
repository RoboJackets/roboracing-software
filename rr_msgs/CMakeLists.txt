cmake_minimum_required(VERSION 3.0.2)
project(rr_msgs)

find_package(catkin REQUIRED COMPONENTS
        gencpp
        genpy
        message_generation
        std_msgs
        dynamic_reconfigure
        )

add_message_files(
        FILES
        speed.msg
        steering.msg
        race_reset.msg
        camera_pose.msg
        chassis_state.msg
        axes.msg
        hsv_tuned.msg
        urc_sign.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

generate_dynamic_reconfigure_options(
        cfg/ColorFilter.cfg
)

catkin_package(
        CATKIN_DEPENDS message_runtime
)
