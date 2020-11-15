#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cstdlib>
#include <thread>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

cv_bridge::CvImagePtr cv_ptr;
ros::Publisher pub;
ros::Publisher pubMove;

std_msgs::String moveMsg;

int roi_x;
int roi_y;
int roi_width;
int roi_height;

cv::Mat sign_forward;
cv::Mat sign_left;
cv::Mat sign_right;
cv::Mat resized;
cv::Mat edges;
cv::Mat result;

double cannyThresholdLow;
double cannyThresholdHigh;

double minVal, curValF, curValR, curValL;
cv::Point minPos, curPosF, curPosR, curPosL;

double templateThresholdMin;

const std::string RIGHT("right");
const std::string LEFT("left");
const std::string STRAIGHT("straight");
const std::string NONE("none");

struct estimate {
    std::string direction;
    double maxVal;
    double maxR;
    cv::Point maxPos;
};

std::map<std::string, int> arrowCount = { {RIGHT, 0}, {LEFT, 0}, {STRAIGHT, 0}};

std::vector<double> scales{1.6, 1.4, 1.2, 1};

/*
 * This sign detector uses multi-scale template matching
 * to find arrows
 */
void sign_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;

    cv::Mat crop;
    if (roi_x == -1 || roi_y == -1 || roi_width == -1 || roi_height == -1) {
        crop = frame;
    } else {
        cv::Rect roi(roi_x, roi_y, roi_width, roi_height);
        crop = frame(roi).clone();
    }

    cv::cvtColor(crop, crop, CV_BGR2GRAY);

    estimate curBest = {NONE, 0, 0, cv::Point()};

    // ROS_INFO_STREAM(std:: endl << "New frame");

    for (double scale: scales) {
        cv::resize(crop, resized, cv::Size(crop.cols * scale, crop.rows * scale));
        double r = 1 / scale;

        if (resized.rows < sign_forward.rows || resized.cols < sign_forward.cols) {
            break;
        }

        cv::Canny(resized, edges, cannyThresholdLow, cannyThresholdHigh);

        auto f1 = []() {
            cv::matchTemplate(edges, sign_left, result, CV_TM_CCOEFF_NORMED);
            cv::minMaxLoc(result, &minVal, &curValL, &minPos, &curPosL);
        };

        auto f2 = []() {
            cv::matchTemplate(edges, sign_right, result, CV_TM_CCOEFF_NORMED);
            cv::minMaxLoc(result, &minVal, &curValR, &minPos, &curPosR);
        };

        auto f3 = []() {
            cv::matchTemplate(edges, sign_forward, result, CV_TM_CCOEFF_NORMED);
            cv::minMaxLoc(result, &minVal, &curValF, &minPos, &curPosF);
        };

        std::thread th1(f1);
        std::thread th2(f2);
        std::thread th3(f3);

        th1.join();
        th2.join();
        th3.join();

        if (curValL > curValF && curValL > curValR && curValL > curBest.maxVal && curValL > templateThresholdMin) {
            curBest.direction = LEFT;
            curBest.maxVal = curValL;
            curBest.maxPos = curPosL;
            curBest.maxR = r;
        } else if (curValR > curValF && curValR > curValL && curValR > curBest.maxVal && curValR > templateThresholdMin) {
            curBest.direction = RIGHT;
            curBest.maxVal = curValR;
            curBest.maxPos = curPosR;
            curBest.maxR = r;
        } else if (curValF > curValL && curValF > curValR && curValF > curBest.maxVal && curValF > templateThresholdMin) {
            curBest.direction = STRAIGHT;
            curBest.maxVal = curValF;
            curBest.maxPos = curPosF;
            curBest.maxR = r;
        }

        //ROS_INFO_STREAM(resized.cols << " " << resized.rows << " " << sign_forward.cols << " " << sign_forward.rows);
    }

    ROS_INFO_STREAM(curBest.direction << " " << curBest.maxVal << " " << 1 / curBest.maxR);

    cv::Point start, end;
    start.x = std::round(curBest.maxPos.x * curBest.maxR);
    start.y = std::round(curBest.maxPos.y * curBest.maxR);
    end.x = std::round((curBest.maxPos.x + sign_left.cols) * curBest.maxR);
    end.y = std::round((curBest.maxPos.y + sign_left.rows) * curBest.maxR);
    cv::rectangle(edges, start, end, (0, 0, 255), 2);

    if (pub.getNumSubscribers() > 0) {
        sensor_msgs::Image out;
        cv_ptr->image = edges;
        cv_ptr->encoding = "mono8";
        cv_ptr->toImageMsg(out);
        pub.publish(out);
    }
}

// loads images, scales as need be, and makes the rotated versions
void loadSignImages(std::string packageName, std::string fileName) {
    std::string path = ros::package::getPath(packageName);
    sign_forward = cv::imread(path + fileName);
    ROS_ERROR_STREAM_COND(sign_forward.empty(), "Arrow sign image not found at " << path + fileName);

    cv::cvtColor(sign_forward, sign_forward, CV_RGB2GRAY);
    cv::resize(sign_forward, sign_forward, cv::Size(sign_forward.rows / 30, sign_forward.cols / 30));
    cv::Canny(sign_forward, sign_forward, 100, 300);  // get the edges as a binary, thresholds don't matter here

    cv::Point2f center(sign_forward.rows / 2, sign_forward.cols / 2);
    cv::Mat rotateLeft = cv::getRotationMatrix2D(center, 90, 1);
    cv::warpAffine(sign_forward, sign_left, rotateLeft, cv::Size(sign_forward.cols, sign_forward.rows));
    cv::flip(sign_left, sign_right, 1);  // 1 = horizontal flip
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sign_detector");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    std::string sign_pub;
    std::string image_sub;
    std::string sign_file_path_from_package;
    std::string sign_file_package_name;

    nhp.param("roi_x", roi_x, 0);
    nhp.param("roi_y", roi_y, 0);
    nhp.param("roi_width", roi_width, -1);  //-1 will default to the whole image
    nhp.param("roi_height", roi_height, -1);

    nhp.param("front_image_subscription", image_sub, std::string("/camera/image_color_rect"));
    nhp.param("sign_file_package_name", sign_file_package_name, std::string("rr_iarrc"));
    nhp.param("sign_file_path_from_package", sign_file_path_from_package, std::string("/src/sign_detector/sign_forward.jpg"));
    nhp.param("sign_string_publisher", sign_pub, std::string("/turn_detected"));

    nhp.param("canny_threshold_low", cannyThresholdLow, 100.0);
    nhp.param("canny_threshold_high", cannyThresholdHigh, 100.0 * 3);
    nhp.param("template_threshold_min", templateThresholdMin, 4.5e+06);

    loadSignImages(sign_file_package_name, sign_file_path_from_package);

    pub = nh.advertise<sensor_msgs::Image>("/sign_detector/signs", 1);  // debug publish of image

    // publish the turn move for Urban Challenge Controller
    pubMove = nh.advertise<std_msgs::String>(sign_pub, 1);
    auto img_real = nh.subscribe(image_sub, 1, sign_callback);

    ros::spin();
    return 0;
}
