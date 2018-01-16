#include <sound_play/sound_play.h>
#include <unistd.h>


int main(int argc, char **argv){
    ros::init(argc, argv, "sound_node");

    ros::NodeHandle nh;
    sound_play::SoundClient sc;

    sleep(1);

    sc.startWaveFromPkg("rr_common", "src/sound_node/fightSong.wav");
    ros::spin();

    sc.stopAll();
}