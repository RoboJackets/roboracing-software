#include <sound_play/sound_play.h>
#include <unistd.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <ros/console.h>

bool raceStarted = false;

void soundCB(const std_msgs::Bool::ConstPtr &bool_msg) {
    raceStarted = bool_msg->data;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "sound_node");
    ros::NodeHandle nh;
    sound_play::SoundClient sc;
    bool lastraceStarted = false;

    ros::Subscriber sub = nh.subscribe("/start_detected", 1, soundCB);
    while (ros::ok()){

        if (raceStarted && !lastraceStarted) {
            ROS_INFO_STREAM("Now playing: Fight Song");
            sleep(1);
            sc.startWaveFromPkg("rr_common", "sounds/fightSong.wav");
        }

        lastraceStarted = raceStarted;
        ros::spinOnce();
    }
    sc.stopAll();
}