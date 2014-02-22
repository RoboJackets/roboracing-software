#include <stdio.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

std::string img_file;
int low_Threshold=10;
int const max_lowThreshold=100;
int CannyThreshold;
int ratio=3;

using namespace cv;
using namespace std;

// ROS image callback
void ImageSaverCB(const sensor_msgs::Image::ConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	Mat blurImg;
	Mat grayscaleImg;
	Mat edgeOp;
	Mat circlesImg;
	//createTrackbar("Min Threshold:","Edge Threshold",&low_Threshold,max_lowThreshold,CannyThreshold);
	ROS_INFO("Received image with %s encoding", msg->encoding.c_str());

	// Convert ROS to OpenCV
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	} catch (cv_bridge::Exception& e) {
		return;
	}

	GaussianBlur(cv_ptr->image,blurImg,Size(11,11),0,0);
	cvtColor(blurImg,grayscaleImg,CV_BGR2GRAY);
	Canny(grayscaleImg,edgeOp,low_Threshold,low_Threshold*ratio,3);
	circlesImg=blurImg.clone();
	vector <Vec3f> circlesdata;
	HoughCircles( grayscaleImg, circlesdata, CV_HOUGH_GRADIENT, 1, grayscaleImg.rows/4, 50, 100, 0, 0 );
	ROS_INFO("I");
	 for( size_t i = 0; i < circlesdata.size(); i++ )
  {	
      Point center(cvRound(circlesdata[i][0]), cvRound(circlesdata[i][1]));
      int radius = cvRound(circlesdata[i][2]);
      circle( circlesImg, center, 3, Scalar(0,255,0), -1, 8, 0 );
      circle( circlesImg, center, radius, Scalar(0,0,255), 3, 8, 0 );
   }

	namedWindow("BlurWindow",WINDOW_AUTOSIZE);
	namedWindow("Original",WINDOW_AUTOSIZE);
	namedWindow("Edges",WINDOW_AUTOSIZE);
	namedWindow("Circles",WINDOW_AUTOSIZE);
	imshow("Circles",circlesImg);
	imshow("Edges",edgeOp);
	imshow("Original", cv_ptr->image);
	imshow("BlurWindow",blurImg);
	waitKey(0);

}

void help(std::ostream& ostr) {
	ostr << "Usage: iarrc_image_saver _img_topic:=<image-topic> _img_file:=<file-name>" << std::endl;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "iarrc_image_display");
	ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // FIXME: Not expected behavior
    if(argc >= 2) {
	    help(std::cerr);
	    exit(1);
    }

    std::string img_topic;
    nhp.param(std::string("img_topic"), img_topic, std::string("/image_raw"));
    nhp.param(std::string("img_file"), img_file, std::string("iarrc_image.png"));

    ROS_INFO("Image topic:= %s", img_topic.c_str());
    ROS_INFO("Image file:= %s", img_file.c_str());

    // Subscribe to ROS topic with callback
    ros::Subscriber img_saver_sub = nh.subscribe(img_topic, 1, ImageSaverCB);


	ROS_INFO("IARRC image saver node ready.");
	ros::spin();
	ROS_INFO("Shutting down IARRC image saver node.");
    return 0;
}
