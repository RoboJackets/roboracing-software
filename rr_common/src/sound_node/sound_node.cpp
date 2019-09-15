#include <ros/console.h>
#include <ros/ros.h>
#include <sound_play/sound_play.h>
#include <std_msgs/Bool.h>
#include <unistd.h>

// https://github.com/ros-drivers/audio_common/issues/96
// To get sound client to play sound file for more than 10 seconds change lines
// 168 and 169 of /opt/ros/[ROSVERSION]/sound_play/soundplay_node.py position =
// self.sound.query_position(Gst.Format.TIME)[1] duration =
// self.sound.query_duration(Gst.Format.TIME)[1]

bool raceStarted = false;

void soundCB(const std_msgs::Bool::ConstPtr &bool_msg) {
    raceStarted = bool_msg->data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sound_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/start_detected", 1, soundCB);

    sound_play::SoundClient sc;

    while (ros::ok()) {
        if (raceStarted) {
            sc.startWaveFromPkg("rr_common", "sounds/fightSong.wav");
            ROS_INFO_STREAM("Now playing: Fight Song");
            sleep(59);  // Duration of song before looping
        }

        ros::spinOnce();
    }
    sc.stopAll();
    return 0;
}