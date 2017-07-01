#include "color_detector.h"
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Header.h>


//using namespace std;
using namespace ros;
using namespace cv;

namespace iarrc {

    using uchar = unsigned char;

    inline bool color_detector::is_blue(const uchar &H, const uchar &S) {
        return (abs(H - 108) < 5) && (S > 50);
    }

    inline bool color_detector::is_orange(const uchar &H, const uchar &S, const uchar &V) {
        return (abs(H - 15) < 5) && (S > 70) && (V > 40);
    }

    inline bool color_detector::is_yellow(const uchar &H, const uchar &S, const uchar &V) {
        return (abs(H - 30) < 0) && (S > 50) && (V > 50);
    }

    inline bool color_detector::is_white(const uchar &H, const uchar &S, const uchar &V) {
        return false;
        // return blue > 220 && green > 220 && red > 220;
        // TODO convert to HSV
    }

    void color_detector::ImageCB(const sensor_msgs::ImageConstPtr &msg) {

        cv_bridge::CvImageConstPtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("CV-Bridge error: %s", e.what());
            return;
        }

        const Mat &frame = cv_ptr->image;

        Mat output = Mat::zeros(frame.rows, frame.cols, CV_8UC3);

        const Mat &frame_masked = frame(mask);

        Mat output_masked = output(mask);

        for (int r = 0; r < frame_masked.rows; r++) {
            const uchar *row = frame_masked.ptr<uchar>(r);
            uchar *out_row = output_masked.ptr<uchar>(r);
            for (int c = 0; c < frame_masked.cols * frame_masked.channels(); c += frame_masked.channels()) {
                const uchar &H = row[c];
                const uchar &S = row[c + 1];
                const uchar &V = row[c + 2];

                if (is_orange(H, S, V)) {
                    out_row[c] = 0;
                    out_row[c + 1] = 127;
                    out_row[c + 2] = 255;
                } else if (is_yellow(H, S, V)) {
                    out_row[c] = 0;
                    out_row[c + 1] = out_row[c + 2] = 255;
                } else if (is_blue(H, S)) {
                    out_row[c] = 255;
                    out_row[c + 1] = out_row[c + 2] = 0;
                }
                if (is_white(H, S, V)) {
                    out_row[c] = out_row[c + 1] = out_row[c + 2] = 255;
                }
            }
        }

        //erode(output_masked, output_masked, erosion_kernel);

        img_pub.publish(cv_bridge::CvImage{std_msgs::Header(), "bgr8", output}.toImageMsg());
    }

    void color_detector::onInit() {
        ROS_INFO("spinning up color detector");
        NodeHandle nh = getNodeHandle();
        image_transport::ImageTransport it(nh);

        mask = Rect(0, 120, 640, 310); // x, y, w, h

        auto kernel_size = 3;
        erosion_kernel = getStructuringElement(MORPH_CROSS, Size(kernel_size, kernel_size));

        image_transport::Subscriber img_saver_sub = it.subscribe("/camera/image_rect", 1, &color_detector::ImageCB, this);
        img_pub = it.advertise("/colors_img", 1);
        spin();

    }

}

PLUGINLIB_EXPORT_CLASS(iarrc::color_detector, nodelet::Nodelet)
