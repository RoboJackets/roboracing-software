#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <tuple>
#include <thread>
#include <mutex>

cv_bridge::CvImagePtr cv_ptr;
ros::Publisher pub;
ros::Publisher pub_move;

std::vector<cv::Mat> templates(3);  // 0 = Forward, 1 = Left, 2 = Right
std_msgs::String move_msg;

int roi_x, roi_y, roi_width, roi_height;
double scalars_start, scalars_end;
int scalars_num;
double template_threshold;

// Credits: https://www.pyimagesearch.com/2015/01/26/multi-scale-template-matching-using-python-opencv/, Daniel Martin
std::tuple<double, cv::Point, int> template_match(const std::vector<cv::Mat> &resized_images, const cv::Mat &template_image) {
    std::tuple<double, cv::Point, int> found = std::make_tuple(-1, cv::Point(-1, -1), -1);
    for(int i = 0; i < resized_images.size(); i++) {
        const cv::Mat &resized_image = resized_images[i];
        if (resized_image.cols < template_image.cols || resized_image.rows < template_image.rows) {
            break;
        }
        cv::Mat result;
        cv::matchTemplate(resized_image, template_image, result, CV_TM_CCOEFF_NORMED);
        double minVal, maxVal;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
        if (std::get<0>(found) == -1 || maxVal > std::get<0>(found)) {
            found = std::make_tuple(maxVal, maxLoc, i);
        }
    }
    return found;
}

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

    cv::resize(crop, crop, cv::Size(int(crop.cols / 2), int(crop.rows / 2)));
    cv::cvtColor(crop, crop, CV_BGR2GRAY);

    std::vector<cv::Mat> resized_images(scalars_num);
    double delta = (scalars_end - scalars_start) / (scalars_start - 1);
    for (int i = 0; i < scalars_num; i++) {
        double scale = scalars_end - i * delta;
        cv::Mat resized_img;
        cv::resize((i == 0) ? crop : resized_images[i-1], resized_img, cv::Size(int(crop.cols * scale), int(crop.rows * scale)));
        resized_images[i] = resized_img;
    }

    std::tuple<double, cv::Point, int> best_found = std::make_tuple(-1, cv::Point(-1, -1), -1);
    int best_template = -1;
    std::mutex best_found_mutex;

    auto worker = [&](int thread_idx) {
        auto found = template_match(resized_images, templates[thread_idx]);
        std::lock_guard lock(best_found_mutex);
        if (std::get<0>(best_found) == -1 || std::get<0>(found) > std::get<0>(best_found)) {
            best_found = found;
            best_template = thread_idx;
        }
    };

    std::vector<std::thread> threads;
    for (int thread_id = 0; thread_id < templates.size(); thread_id++) {
        threads.emplace_back(worker, thread_id);
    }

    for (auto &t : threads) {
        t.join();
    }

    auto[maxVal, maxLoc, best_i] = best_found;
    double r = double(crop.rows) / resized_images[best_i].rows;

    std::vector<std::string> i_to_template = {"FORWARD", "LEFT", "RIGHT"};

    cv::Point start, end;
    start.x = int(maxLoc.x * r);
    start.y = int(maxLoc.y * r);
    end.x = int(start.x + templates[best_template].cols * r);
    end.y = int(start.y + templates[best_template].rows * r);

    cv::cvtColor(crop, crop, CV_GRAY2BGR);
    cv::rectangle(crop, start, end, (maxVal > template_threshold) ? cv::Scalar(0,255,0) : cv::Scalar(255,0,0), 1);

    // ROS_INFO_STREAM(i_to_template[best_template] << " " << maxVal);

    if (pub.getNumSubscribers() > 0) {
        sensor_msgs::Image out;
        cv_ptr->image = crop;
        cv_ptr->encoding = "bgr8";
        cv_ptr->toImageMsg(out);
        pub.publish(out);
    }

    if (pub_move.getNumSubscribers() > 0 && maxVal >= template_threshold) {
        std_msgs::String out;
        out.data = i_to_template[best_template];
        pub_move.publish(out);
    }
}

void load_templates(std::string package_name, std::string file_name) {
    std::string path = ros::package::getPath(package_name);
    templates[0] = cv::imread(path + file_name);
    ROS_ERROR_STREAM_COND(templates[0].empty(), "Arrow sign image not found at " << path + file_name);

    cv::resize(templates[0], templates[0], cv::Size(int(templates[0].rows / 30), int(templates[0].cols / 30)));
    cv::cvtColor(templates[0], templates[0], CV_RGB2GRAY);

    cv::Point2f center(templates[0].rows / 2, templates[0].cols / 2);
    cv::Mat rotateLeft = cv::getRotationMatrix2D(center, 90, 1);
    cv::warpAffine(templates[0], templates[1], rotateLeft, cv::Size(templates[0].cols, templates[0].rows));
    cv::flip(templates[1], templates[2], 1);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sign_detector");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    std::string sign_pub;
    std::string image_sub;
    std::string sign_file_path_from_package;
    std::string sign_file_package_name;

    nhp.param("front_image_subscription", image_sub, std::string("/camera/image_color_rect"));
    nhp.param("sign_file_package_name", sign_file_package_name, std::string("rr_iarrc"));
    nhp.param("sign_file_path_from_package", sign_file_path_from_package, std::string("/src/sign_detector/sign_forward.jpg"));
    nhp.param("sign_string_publisher", sign_pub, std::string("/turn_detected"));

    nhp.param("roi_x", roi_x, 0);
    nhp.param("roi_y", roi_y, 0);
    nhp.param("roi_width", roi_width, -1);  //-1 will default to the whole image
    nhp.param("roi_height", roi_height, -1);

    nhp.param("template_confidence_threshold", template_threshold, 0.5);
    nhp.param("scalars_start", scalars_start, 0.6);
    nhp.param("scalars_end", scalars_end, 1.2);
    nhp.param("scalars_num", scalars_num, 4);

    load_templates(sign_file_package_name, sign_file_path_from_package);

    pub = nh.advertise<sensor_msgs::Image>("/sign_detector/signs", 1);  // debug publish of image

    pub_move = nh.advertise<std_msgs::String>(sign_pub, 1);
    auto img_real = nh.subscribe(image_sub, 1, sign_callback);

    ros::spin();
    return 0;
}


