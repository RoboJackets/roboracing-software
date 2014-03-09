#include <stdio.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <Eigen/Dense>

std::string img_file;
int low_Threshold=50;
int const max_lowThreshold=100;
int CannyThreshold;
int ratio=2;
int sigma=0;
int brightThreshold=80;
ros::Publisher img_pub;
sensor_msgs::Image rosimage;

using namespace cv;
using namespace std;

bool goodCircle(Vec3f &info,Mat &img);

// ROS image callback
void ImageSaverCB(const sensor_msgs::Image::ConstPtr& msg) {
	
	//cout<<"Sigma:";
	//cin>>sigma;
	cv_bridge::CvImagePtr cv_ptr;
	Mat blurImg;
	Mat downBlurImg;
	Mat grayscaleImg;
	Mat downGrayscaleImg;
	Mat edgeOp;
	Mat downEdgeOp;
	Mat circlesImg;
	Mat kernel=getGaussianKernel(15,sigma);
	//createTrackbar("Min Threshold:","Edge Threshold",&low_Threshold,max_lowThreshold,CannyThreshold);
	//ROS_INFO("Received image with %s encoding", msg->encoding.c_str());

	// Convert ROS to OpenCV
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	} catch (cv_bridge::Exception& e) {
		return;
	}

	//Eigen::Map<Eigen::MatrixXd> eigenT(kernel.ptr<double>(),kernel.rows,kernel.cols);
	//cout<<eigenT;
	vector <Vec3f> circlesdata;
	GaussianBlur(cv_ptr->image,blurImg,Size(9,9),sigma,sigma);
	//pyrDown(blurImg,downBlurImg,Size(blurImg.cols/2,blurImg.rows/2));
	cvtColor(blurImg,grayscaleImg,CV_BGR2GRAY);
	//cvtColor(downBlurImg,downGrayscaleImg,CV_BGR2GRAY);
	Canny(grayscaleImg,edgeOp,low_Threshold,low_Threshold*ratio,3);
	//Canny(downGrayscaleImg,downEdgeOp,low_Threshold,low_Threshold*ratio,3);
	circlesImg=blurImg.clone();
	HoughCircles( grayscaleImg, circlesdata, CV_HOUGH_GRADIENT, 1, grayscaleImg.rows/4, low_Threshold*2, 50, 0, 0 );
	//ROS_INFO("I");

	 for( size_t i = 0; i < circlesdata.size(); i++ )
  {	
      if (goodCircle(circlesdata [i],grayscaleImg))
      { 
      		Point center(cvRound(circlesdata[i][0]), cvRound(circlesdata[i][1]));
      		int radius = cvRound(circlesdata[i][2]);
      		circle( circlesImg, center, 3, Scalar(0,255,0), -1, 8, 0 );
      		circle( circlesImg, center, radius, Scalar(127,0,127), 2, 8, 0 );
      	}
    /*  else
      {
      		Point center(cvRound(circlesdata[i][0]), cvRound(circlesdata[i][1]));
      		int radius = cvRound(circlesdata[i][2]);
      		circle( circlesImg, center, 3, Scalar(255,255,255), -1, 8, 0 );
      		circle( circlesImg, center, radius, Scalar(0,0,0), 3, 8, 0 );
      	}
      */
   }

   cv_ptr->image=circlesImg;
   cv_ptr->encoding="bgr8";
   cv_ptr->toImageMsg(rosimage);
   img_pub.publish(rosimage);


   //ROS_INFO("I");

   	/*
   	//namedWindow("Gaussian Kernel",WINDOW_AUTOSIZE);
	namedWindow("BlurWindow",WINDOW_AUTOSIZE);
	//namedWindow("Original",WINDOW_AUTOSIZE);
	//namedWindow("Downsampled Edges",WINDOW_AUTOSIZE);
	namedWindow("Edges",WINDOW_AUTOSIZE);
	namedWindow("Circles",WINDOW_AUTOSIZE);
	//imshow("Gaussian Kernel",kernel);
	imshow("Circles",circlesImg);
	imshow("Edges",edgeOp);
	//imshow("Downsampled Edges",downEdgeOp);
	//imshow("Original", cv_ptr->image);
	imshow("BlurWindow",grayscaleImg);
	waitKey(0);
	*/


}

void help(std::ostream& ostr) {
	ostr << "Usage: iarrc_image_display _img_topic:=<image-topic> _img_file:=<file-name>" << std::endl;
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
    img_pub = nh.advertise<sensor_msgs::Image>("/image_circles", 1);


	ROS_INFO("IARRC image saver node ready.");
	ros::spin();
	ROS_INFO("Shutting down IARRC image saver node.");
    return 0;
}



bool goodCircle(Vec3f &info,Mat &img)
{
	int centerx=info[0];
	int centery=info[1];
	float radius=info[2];
	//ROS_INFO("x:%d\ty:%d\tradius:%f",centerx,centery,radius);
	double goodPoints=0;
	double totalPoints=0;
	//ROS_INFO("Test Value:%d",img.at<uchar>(centery,centerx));
	
	for (int i = 0; i < img.cols; i++)
	{
		for (int j = 0; j < img.rows; j++)
		{
			double dist=pow((pow((centerx-i),2)+pow((centery-j),2)),(0.5));
			if (dist<=radius)
			{
				if (img.at<uchar>(j,i)>240)
				{	
					goodPoints++;
				}
			}
			totalPoints++;
		}
	}

	totalPoints=3.14*pow(radius,2);
	double percent=goodPoints/totalPoints*100;

	//ROS_INFO("Good:%f, Total:%f, Ratio:%f",goodPoints,totalPoints,percent);

	if (percent>=brightThreshold)
	{
		return true;
	}
	else
	{
		return false;
	}
}
