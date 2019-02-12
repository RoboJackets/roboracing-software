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
    cv::Mat frame = cv_ptr->image;

    cv::Mat crop;
    if (roi_width == -1 || roi_height == -1) {
      crop = frame;
    } else {
      cv::Rect roi(roi_x, roi_y, roi_width, roi_height);
      crop = frame(roi); //note that crop is just a reference to that roi of the frame
    }
    cv::GaussianBlur(crop, crop, cv::Size(5,5), 0, 0, cv::BORDER_DEFAULT);

    //#TODO: matchTemplate and minMaxLoc stuff



    if (pub.getNumSubscribers() > 0) {
        sensor_msgs::Image outmsg;
        cv_ptr->image = crop;
        cv_ptr->encoding = "bgr8";
        cv_ptr->toImageMsg(outmsg);
        pub.publish(outmsg);
    }
}


//loads images, scales as need be, and makes the rotated versions
int loadSignImages(std::string packageName, std::string fileName) {
  std::string path = ros::package::getPath(packageName);
  sign_forward = cv::imread(path + fileName);
  //#TODO: scale the sign if need be
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

    nhp.param("roi_x", roi_x, 0);
    nhp.param("roi_y", roi_y, 0);
    nhp.param("roi_width", roi_width, -1);
    nhp.param("roi_height", roi_height, -1);
    nhp.param("img_subscription", image_sub, std::string("/camera/image_color_rect"));
    nhp.param("sign_file_package_name", sign_file_package_name, std::string("rr_iarrc"));
    nhp.param("sign_file_path_from_package", sign_file_path_from_package, std::string("/src/sign_detector/sign_forward.jpg"));

    loadSignImages(sign_file_package_name, sign_file_path_from_package);


    pub = nh.advertise<sensor_msgs::Image>("/signs", 1); //test publish of image
    auto img_real = nh.subscribe(image_sub, 1, img_callback);

    ros::spin();
    return 0;
}
