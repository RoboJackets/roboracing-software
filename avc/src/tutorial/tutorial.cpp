#include <ros/ros.h>
#include <ros/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

ros::Publisher pub;

void img_callback(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;

    // get binary image of red-barrel-colored things
    cv::Mat output(frame.rows, frame.cols, CV_8UC1);
    auto channels = frame.channels();
    for(auto i = 0; i < frame.rows; i++) {
        const unsigned char* row =  frame.ptr<unsigned char>(i);
        unsigned char* out_row   = output.ptr<unsigned char>(i);

        for(auto j = 0; j < frame.cols; j++) {
            auto b = row[j*channels];
            auto g = row[j*channels+1];
            float r = (float) row[j*channels+2];
            if( r > 50  &&  r/(r+g+b) > 0.5) {
                out_row[j] = 255;
            } else {
                out_row[j] = 0;
            }
        }
    }
    

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
