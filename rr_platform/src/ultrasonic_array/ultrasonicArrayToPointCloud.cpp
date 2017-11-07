#include <ros/ros.h>
#include <ros/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

std::string readLine(boost::asio::serial_port &port) {
    std::string line = "";
    while (true) {
        char in;
        try {
            boost::asio::read(port, boost::asio::buffer(&in, 1));
        } catch (
                boost::exception_detail::clone_impl <boost::exception_detail::error_info_injector<boost::system::system_error>> &err) {
            ROS_ERROR("Error reading serial port.");
            ROS_ERROR_STREAM(err.what());
            return line;
        }
        if (in == '\n')
            return line;
        if (in == '\r')
            return line;
        line += in;
    }
}




ros::Publisher pub;

void img_callback(const sensor_msgs::ImageConstPtr& msg) {


    sensor_msgs::Image outmsg;
    cv_ptr->image = output;
    cv_ptr->encoding = "mono8";
    cv_ptr->toImageMsg(outmsg);

    pub.publish(outmsg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "tutorial");

	ros::NodeHandle nh;
	pub = nh.advertise<sensor_msgs::Image>("/tutorial_image", 1);
	auto img_sub = nh.subscribe("/camera/image_rect", 1, img_callback);

	ros::spin();
	return 0;
}
