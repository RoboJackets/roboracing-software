#include <ros/ros.h>
#include <rr_platform/chassis_state.h>
#include <rosbag/recorder.h>
#include <boost/algorithm/string.hpp>

/*
 * Remote control for recording Rosbag files.
 * Uses a button from E-Stop remote to start recording bag files.
 *
 * @author Brian Cochran @btdubs
*/

bool startRecording = false;
bool begunRecording = false;

void chassisStateCallback(const rr_platform::chassis_state::ConstPtr &chassis_msg) {
    startRecording = chassis_msg->record_bag;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rosbag_remote_recorder");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/chassis_state", 1, chassisStateCallback);

    ros::NodeHandle nhp("~");
    std::string topics_to_record;
    std::string bag_name_prefix;
    std::string folder_path;
    int bag_max_size;
    nhp.param(std::string("topics_to_record"), topics_to_record, std::string(""));
    nhp.param(std::string("bag_name_prefix"), bag_name_prefix, std::string(""));
    nhp.param(std::string("folder_path"), folder_path, std::string(""));
    nhp.param(std::string("bag_max_size"), bag_max_size, 3000);

    std::vector< std::string > topics_list;
    boost::split(topics_list, topics_to_record, boost::is_any_of(" "));


    rosbag::RecorderOptions recorder_options_;
    recorder_options_.record_all = false;
    recorder_options_.topics = topics_list; //only record specified topics
    recorder_options_.prefix = folder_path + bag_name_prefix;
    recorder_options_.trigger = false;
    recorder_options_.regex = false;
    recorder_options_.do_exclude = false;
    recorder_options_.quiet = false;
    recorder_options_.append_date = true;
    recorder_options_.verbose = false;
    recorder_options_.buffer_size = 0; //0 is unlimited. May want to limit
    recorder_options_.chunk_size = 1024 * 768;
    recorder_options_.limit = 0; // 0 is unlimited
    recorder_options_.split = true;
    recorder_options_.max_size = bag_max_size * 1024 * 1024; //split after this size
    recorder_options_.min_space = bag_max_size * 1024 * 1024;
    recorder_options_.min_space_str = "Recording space is running out";
    rosbag::Recorder recorder(recorder_options_);

    while (ros::ok()){
        if (startRecording && !begunRecording) {
            recorder.run();
            begunRecording = true;
            ROS_INFO_STREAM("Recording bag to " + folder_path + bag_name_prefix);
        }
        if (!startRecording && begunRecording) {
            ROS_INFO_STREAM("Stopping bag recording");
            ros::shutdown(); //only way to stop the recording, but we respawn!
        }

        ros::spinOnce();
    }
    return 0;
}
