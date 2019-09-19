cmake_minimum_required(VERSION 2.8.3)
project(rr_msgs)

find_package(catkin REQUIRED COMPONENTS
        gencpp
        genpy
        message_generation
        std_msgs
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

catkin_package(
        CATKIN_DEPENDS message_runtime
)
