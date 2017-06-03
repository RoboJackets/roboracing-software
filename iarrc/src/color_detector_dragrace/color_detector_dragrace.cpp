#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace ros;
using namespace cv;

using uchar = unsigned char;

Publisher img_pub;

Rect mask;

Mat erosion_kernel;

Mat findWhiteLines(const Mat& image) {
    Mat frame;
    image.copyTo(frame);
    frame *= 1.25;

    for(int r = 0; r < frame.rows; r++) {
        uchar* row = frame.ptr<uchar>(r);
        for(int c = 0; c < frame.cols * frame.channels(); c+= frame.channels()) {
            uchar& blue = row[c];
            uchar& green = row[c+1];
            uchar& red = row[c+2];

            if(blue > 220 && green > 220 && red > 220) {
                blue = green = red = 255;
            } else {
                blue = green = red = 0;
            }
        }
    }
    return frame;
}

Mat findYellowLines(const Mat& image) {
    Mat frame;
    image.copyTo(frame);
    
    cvtColor(frame, frame, CV_BGR2HSV);

    Mat output(image.rows, image.cols, CV_8UC3);

    for(int r = 0; r < frame.rows; r++) {
        uchar* row = frame.ptr<uchar>(r);
        uchar* out_row = output.ptr<uchar>(r);
        for(int c = 0; c < frame.cols * frame.channels(); c+= frame.channels()) {
            uchar& H = row[c];
            uchar& S = row[c+1];
            uchar& V = row[c+2];

            //if(abs(H - 30) < 10 && S > 50 && V > 30) {
            if(abs(H - 30) < 0 && S > 50 && V > 50) {       //No yellow lines to be detected in drag race (<0 never true)
                out_row[c] = 0;
                out_row[c+1] = out_row[c+2] = 255;
            } else {
                out_row[c] = out_row[c+1] = out_row[c+2] = 0;
            }
        }
    }

    auto kernel_size = 3;
    Mat erosion_kernel = getStructuringElement(MORPH_CROSS, Size(kernel_size, kernel_size));

    erode(output, output, erosion_kernel);

    return output;
}

Mat findOrange(const Mat& image) {
    Mat frame;
    image.copyTo(frame);

    cvtColor(frame, frame, CV_BGR2HSV);

    Mat output(image.rows, image.cols, CV_8UC3);

    for(int r = 0; r < frame.rows; r++) {
        uchar* row = frame.ptr<uchar>(r);
        uchar* out_row = output.ptr<uchar>(r);
        for(int c = 0; c < frame.cols * frame.channels(); c+= frame.channels()) {
            uchar& H = row[c];
            uchar& S = row[c+1];
            uchar& V = row[c+2];

            if(abs(H - 15) < 5 && S > 70 && V > 40) {
                out_row[c] = 0;
                out_row[c+1] = 127;
                out_row[c+2] = 255;
            } else {
                out_row[c] = out_row[c+1] = out_row[c+2] = 0;
            }
        }
    }

    auto kernel_size = 11;
    Mat erosion_kernel = getStructuringElement(MORPH_CROSS, Size(kernel_size, kernel_size));

    erode(output, output, erosion_kernel);

    return output;
}

Mat findBlueLines(const Mat& image) {
    Mat frame;
    image.copyTo(frame);

    image *= 1.2;

    cvtColor(frame, frame, CV_BGR2HSV);

    Mat output(image.rows, image.cols, CV_8UC3);

    for(int r = 0; r < frame.rows; r++) {
        uchar* row = frame.ptr<uchar>(r);
        uchar* out_row = output.ptr<uchar>(r);
        for(int c = 0; c < frame.cols * frame.channels(); c+= frame.channels()) {
            uchar& H = row[c];
            uchar& S = row[c+1];
            uchar& V = row[c+2];

            if(abs(H - 108) < 5 && S > 50) {
                out_row[c] = 255;
                out_row[c+1] = out_row[c+2] = 0;
            } else {
                out_row[c] = out_row[c+1] = out_row[c+2] = 0;
            }
        }
    }

    return output;
}

inline bool is_blue(const uchar &H, const uchar &S) {
    return ( abs(H - 108) < 5 ) && ( S > 50 );
}

inline bool is_orange(const uchar &H, const uchar &S, const uchar &V) {
    return ( abs(H - 15) < 5 ) && ( S > 70 ) && ( V > 40 );
}

inline bool is_yellow(const uchar &H, const uchar &S, const uchar &V) {
    return false;
    // return ( abs(H - 30) < 0 ) && ( S > 50 ) && ( V > 50 );
    // No yellow lines to be detected in drag race (<0 never true)
}

inline bool is_white(const uchar &H, const uchar &S, const uchar &V) {
    return false; 
    // return blue > 220 && green > 220 && red > 220;
    // TODO convert to HSV
}

void ImageCB(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImageConstPtr cv_ptr;
	
	try {
		cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("CV-Bridge error: %s", e.what());
		return;
	}
	
	const Mat &frame = cv_ptr->image;
	
	Mat output = Mat::zeros(frame.rows, frame.cols, CV_8UC3);
	
	const Mat &frame_masked = frame(mask);
	
	Mat output_masked = output(mask);
	
	for(int r = 0; r < frame_masked.rows; r++) {
	    const uchar *row = frame_masked.ptr<uchar>(r);
	    uchar *out_row = output_masked.ptr<uchar>(r);
	    for(int c = 0; c < frame_masked.cols * frame_masked.channels(); c += frame_masked.channels()) {
	        const uchar &H = row[c];
	        const uchar &S = row[c+1];
	        const uchar &V = row[c+2];
	        
	        if(is_orange(H,S,V)) {
	            out_row[c] = 0;
                out_row[c+1] = 127;
                out_row[c+2] = 255;
	        } else if(is_yellow(H,S,V)) {
	            out_row[c] = 0;
                out_row[c+1] = out_row[c+2] = 255;
	        } else if(is_blue(H,S)) {
	            out_row[c] = 255;
	            out_row[c+1] = out_row[c+2] = 0;
	        } if(is_white(H,S,V)) {
	            out_row[c] = out_row[c+1] = out_row[c+2] = 255;
	        }
	    }
	}

    //erode(output_masked, output_masked, erosion_kernel);

	img_pub.publish(cv_bridge::CvImage{std_msgs::Header(), "bgr8", output}.toImageMsg());
}

int main(int argc, char** argv) {

    init(argc, argv, "color_detector_dragrace");

    NodeHandle nh;
    
    //drag race
    
    //vector<Mat> mask_segments = {
    //    Mat::zeros(120,640,CV_8UC3),
    //    Mat(310,640,CV_8UC3, CV_RGB(1,1,1)),
    //    Mat::zeros(50,640,CV_8UC3)
    //};
    //vconcat(mask_segments, mask);
    
    mask = Rect(0, 120, 640, 310); // x, y, w, h
    
    auto kernel_size = 3;
    erosion_kernel = getStructuringElement(MORPH_CROSS, Size(kernel_size, kernel_size));


    ros::Subscriber img_saver_sub = nh.subscribe("/camera/image_rect", 1, ImageCB);
	
	img_pub = nh.advertise<sensor_msgs::Image>(string("/colors_img"), 1);
        
    spin();

    return 0;

}
