#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stdlib.h>


cv_bridge::CvImagePtr cv_ptr;
ros::Publisher pub;

int roi_x;
int roi_y;
int roi_width;
int roi_height;

cv::Mat sign_forward;
cv::Mat sign_left;
cv::Mat sign_right;


void img_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    //cv::Mat frame = cv_ptr->image;

    //########################################FOR TEST ONLY #TODO REMOVE THIS and uncomment above#######
    std::string path2 = ros::package::getPath("rr_iarrc");
    cv::Mat frame = cv::imread(path2 + "/src/sign_detector/traffic_signs1.png");
    //###############################33

    cv::Mat gray;
    cv::cvtColor(frame, gray, CV_RGB2GRAY);
    equalizeHist( gray, gray ); //contrast enhancement

    cv::Mat crop_gray;
    cv::Mat crop_color;
    if (roi_width == -1 || roi_height == -1) {
      crop_gray = gray;
      crop_color = frame;
    } else {
      cv::Rect roi(roi_x, roi_y, roi_width, roi_height);
      crop_gray = gray(roi); //note that crop is just a reference to that roi of the frame, not a copy
      crop_color = frame(roi);
    }
    //cv::GaussianBlur(crop_gray, crop_gray, cv::Size(5,5), 0, 0, cv::BORDER_DEFAULT); //may or may not help with template matching

    //#TODO: matchTemplate and minMaxLoc stuff
    cv::Mat result;
double start =ros::Time::now().toSec();
for (int i = 0; i < 8; i++) {
    cv::matchTemplate( crop_gray, sign_forward, result, CV_TM_SQDIFF_NORMED );
    cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
}
    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
    cv::Point matchLoc;

    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
    matchLoc = minLoc; //@Note CV_TM_SQDIFF_NORMED uses the min
double end =ros::Time::now().toSec();
ROS_INFO_STREAM(end-start);


    //Some debug images for us
    //show where the match is found
    cv::Mat sign = sign_forward;
    cv::rectangle(crop_color, matchLoc, cv::Point(matchLoc.x + sign.cols, matchLoc.y + sign.rows), cv::Scalar(0, 255, 0), 2, 8, 0);
    //show where we are cropped to
    cv::rectangle(crop_color, cv::Point(0,0), cv::Point(crop_color.rows-1, crop_color.cols-1), cv::Scalar(0,255,0), 2, 8 ,0);

    if (pub.getNumSubscribers() > 0) {
        sensor_msgs::Image outmsg;
        cv_ptr->image = frame;
        cv_ptr->encoding = "bgr8";
        cv_ptr->toImageMsg(outmsg);
        pub.publish(outmsg);
    }
}


//loads images, scales as need be, and makes the rotated versions
int loadSignImages(std::string packageName, std::string fileName) {
  //#TODO: ADD SOME ERROR IN LOADING DETECTION!
  std::string path = ros::package::getPath(packageName);
  sign_forward = cv::imread(path + fileName);
  cv::cvtColor(sign_forward, sign_forward, CV_RGB2GRAY); //single channel helps with templateMatch
  //currently sign is 2000x2000px
  cv::resize(sign_forward, sign_forward, cv::Size(300,300));//110,110)); //scale image down #TODO: more options


  cv::Point2f center(sign_forward.rows/2, sign_forward.cols/2);
  cv::Mat rotateLeft = cv::getRotationMatrix2D(center, 90, 1);
  cv::warpAffine(sign_forward, sign_left, rotateLeft, cv::Size(sign_forward.cols, sign_forward.rows));
  cv::flip(sign_left, sign_right, 1); //1 = horizontal flip

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "sign_detector");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string image_sub;
    std::string sign_file_path_from_package;
    std::string sign_file_package_name;

    nhp.param("roi_x", roi_x, 300);//0);
    nhp.param("roi_y", roi_y, 300);//0);
    nhp.param("roi_width", roi_width, 600);//-1); //-1 will default to the whole image
    nhp.param("roi_height", roi_height, 600);//-1);
    nhp.param("img_subscription", image_sub, std::string("/camera/image_color_rect"));
    nhp.param("sign_file_package_name", sign_file_package_name, std::string("rr_iarrc"));
    nhp.param("sign_file_path_from_package", sign_file_path_from_package, std::string("/src/sign_detector/sign_forward.jpg"));

    loadSignImages(sign_file_package_name, sign_file_path_from_package);


    pub = nh.advertise<sensor_msgs::Image>("/signs", 1); //test publish of image
    auto img_real = nh.subscribe(image_sub, 1, img_callback);

    ros::spin();
    return 0;
}
