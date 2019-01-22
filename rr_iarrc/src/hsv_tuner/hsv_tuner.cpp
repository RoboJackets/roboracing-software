#include <ros/ros.h>
#include <rr_iarrc/hsv_tuned.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

ros::Publisher hsv_pub;
rr_iarrc::hsv_tuned hsv_msg;

const int hsv_slider_max = 255;

int32_t white_h_low_slider;     
int32_t white_s_low_slider;
int32_t white_v_low_slider; 
int32_t white_h_high_slider;        
int32_t white_s_high_slider;
int32_t white_v_high_slider;
// Callback for trackbar
void on_trackbar( int, void* ){
    hsv_msg.white_h_low = white_h_low_slider;
    hsv_msg.white_s_low = white_s_low_slider;
    hsv_msg.white_v_low = white_v_low_slider;
    
    hsv_msg.white_h_high = white_h_high_slider;
    hsv_msg.white_s_high = white_s_high_slider;
    hsv_msg.white_v_high = white_v_high_slider;

    rr_iarrc::hsv_tuned publishable_copy = hsv_msg;
    publishable_copy.header.stamp = ros::Time::now();
    hsv_pub.publish(publishable_copy);
}



int main(int argc, char **argv) {

    ros::init(argc, argv, "hsv_tuner");
    ros::NodeHandle handle;
    ros::NodeHandle private_handle("~");
    
    hsv_pub = handle.advertise<rr_iarrc::hsv_tuned>("/hsv_tuned", 1);

    rr_iarrc::hsv_tuned hsv_tuned_msg;
    hsv_tuned_msg.header.frame_id = "hsv_tuned";
    ROS_INFO("Running HSV tuner panel.");

    /// Initialize values
    white_h_low_slider = 255;
    white_s_low_slider = 255;
    white_v_low_slider = 255;
    white_h_high_slider = 255;
    white_s_high_slider = 255;
    white_v_high_slider = 255;

    hsv_msg.white_h_low = white_h_low_slider;
    hsv_msg.white_s_low = white_s_low_slider;
    hsv_msg.white_v_low = white_v_low_slider;
    
    hsv_msg.white_h_high = white_h_high_slider;
    hsv_msg.white_s_high = white_s_high_slider;
    hsv_msg.white_v_high = white_v_high_slider;

    /// Create Windows
   namedWindow("HSV Tuner", 1);

    /// Create Trackbars
    char white_h_low[50];
    sprintf( white_h_low, "White Hue Low: %d", hsv_slider_max);
    
    char white_s_low[50];
    sprintf( white_s_low, "White Saturation Low: %d", hsv_slider_max);
    char white_v_low[50];
    sprintf( white_v_low, "White Value Low: %d", hsv_slider_max);
    char white_h_high[50];
    sprintf( white_h_high, "White Hue High: %d", hsv_slider_max);
    char white_s_high[50];
    sprintf( white_s_high, "White Saturation High: %d", hsv_slider_max);
    char white_v_high[50];
    sprintf( white_v_high, "White Value High: %d", hsv_slider_max);

    createTrackbar(white_h_low, "HSV Tuner", &white_h_low_slider, hsv_slider_max, on_trackbar);
    createTrackbar(white_s_low, "HSV Tuner", &white_s_low_slider, hsv_slider_max, on_trackbar);
    createTrackbar(white_v_low, "HSV Tuner", &white_v_low_slider, hsv_slider_max, on_trackbar);
    createTrackbar(white_h_high, "HSV Tuner", &white_h_high_slider, hsv_slider_max, on_trackbar);
    createTrackbar(white_s_high, "HSV Tuner", &white_s_high_slider, hsv_slider_max, on_trackbar);
    createTrackbar(white_v_high, "HSV Tuner", &white_v_high_slider, hsv_slider_max, on_trackbar);
    
    /// Show some stuff
    //on_trackbar(alpha_slider, 0 );



    while(ros::ok()) {
        ros::spinOnce();        
      
        waitKey(1);



        

    }

    return 0;
}