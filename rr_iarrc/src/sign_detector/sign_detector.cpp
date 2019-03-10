#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stdlib.h>


cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImagePtr cv_ptrLine;
ros::Publisher pub;
ros::Publisher pubLine;
ros::Publisher pubMove;

int roi_x;
int roi_y;
int roi_width;
int roi_height;

cv::Mat sign_forward;
cv::Mat sign_left;
cv::Mat sign_right;

std::vector<cv::Point> template_contour_upright;
double minContourArea;
double straightMatchSimilarityThreshold;
double turnMatchSimilarityThreshold;

double cannyThresholdLow;
double cannyThresholdHigh;

std::string currentMove = "NONE";

void sign_callback(const sensor_msgs::ImageConstPtr& msg) {
ros::Time start = ros::Time::now();
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;

    cv::Mat crop;
    if (roi_x == -1 || roi_y == -1 || roi_width == -1 || roi_height == -1) {
      crop = frame;
    } else {
      cv::Rect roi(roi_x, roi_y, roi_width, roi_height);
      crop = frame(roi); //note that crop is just a reference to that roi of the frame, not a copy
    }

    //cv::GaussianBlur(crop, crop, cv::Size(5,5), 0, 0, cv::BORDER_DEFAULT); //may or may not help Canny

    //Color -> binary; Edge detection
    cv::Mat edges;
    cv::Canny(crop, edges, cannyThresholdLow, cannyThresholdHigh );

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    cv::morphologyEx(edges, edges, cv::MORPH_CLOSE, kernel);

    //Find arrow-like shapes with Contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    cv::drawContours(crop, contours, -1, cv::Scalar(0,255,0), 2); //debug #TODO: REMOVE

    cv::Rect bestMatch(0,0,0,0);
    std::string bestMove = "NONE"; //"right", "left", "straight"

    for(size_t i=0; i<contours.size(); i++) {
      std::vector<cv::Point> c = contours[i]; //curent contour reference
      double perimeter = cv::arcLength(c, true);
      double epsilon = 0.04 * perimeter;
      std::vector<cv::Point> approxC;
      cv::approxPolyDP(c, approxC, epsilon, true);
      if (approxC.size() == 7 && cv::contourArea(approxC) > minContourArea) { //>= 7 || approxC.size() <= 9) { //#TODO: adjust bounds of what is an arrow after adjusting epsilon
          //check the ratio is arrow-like
          cv::Rect rect = cv::boundingRect(c);
          double ratioMin = 1.5; //ratio of width to height or vice versa
          double ratioMax = 2.2;
          if ( (rect.width * ratioMin <= rect.height && rect.width * ratioMax >= rect.height) ||
                (rect.height * ratioMin <= rect.width && rect.height * ratioMax >= rect.width) ) {

            cv::rectangle(crop, rect, cv::Scalar(0,255,255),3); //debug

            if (rect.width > rect.height) {
              //Sideways
              double matchSimilarity = cv::matchShapes(c, template_contour_upright, CV_CONTOURS_MATCH_I1, 0); //#TODO: change to sideways contours
              cv::putText(crop, std::to_string(matchSimilarity), cv::Point(rect.x, rect.y + 25), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(255,0,0), 2);

              if (matchSimilarity <= turnMatchSimilarityThreshold) {
                  // find top point of the arrow and test its x location
                  auto extremeY = std::minmax_element(c.begin(), c.end(), [](cv::Point const& a, cv::Point const& b){
                      return a.y < b.y;
                  });
                  if (extremeY.first->x < rect.x + rect.width/2) {//topmost point is far left
                    //left pointing arrow!
                    cv::putText(crop, "Left", rect.tl(), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255), 2);

                    if (rect.area() > bestMatch.area()) {
                      bestMatch = rect;
                      bestMove = "left";
                    }

                  } else {
                    //right pointing arrow!
                    cv::putText(crop, "Right", rect.tl(), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255), 2);

                    if (rect.area() > bestMatch.area()) {
                      bestMatch = rect;
                      bestMove = "right";
                    }

                  }

              }
            } else {
              //Straight
              double matchSimilarity = cv::matchShapes(approxC, template_contour_upright, CV_CONTOURS_MATCH_I1, 0.0);

              cv::putText(crop, std::to_string(matchSimilarity), cv::Point(rect.x, rect.y +50), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(255,0,0), 2);

              if (matchSimilarity <= straightMatchSimilarityThreshold) {
                  cv::putText(crop, "Straight", rect.tl(), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255), 2);

                  if (rect.area() > bestMatch.area()) {
                    bestMatch = rect;
                    bestMove = "straight";
                  }
              }
            }

        }
      }


    }

    //Some debug images for us
    //show the bestMatch
    cv::rectangle(crop, bestMatch, cv::Scalar(255,0,255), 2, 8 ,0);
    cv::putText(crop, bestMove, bestMatch.tl(), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(255,0,255), 2);

    //show where we are cropped to
    cv::rectangle(crop, cv::Point(0,0), cv::Point(crop.cols-1, crop.rows-1), cv::Scalar(0,255,0), 2, 8 ,0);

    if (pub.getNumSubscribers() > 0) {
        sensor_msgs::Image outmsg;
        cv_ptr->image = frame;
        cv_ptr->encoding = "bgr8";
        cv_ptr->toImageMsg(outmsg);
        pub.publish(outmsg);
    }
}




void stopBar_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_ptrLine = cv_bridge::toCvCopy(msg, "mono8");
    cv::Mat frame = cv_ptrLine->image;

	bool stopBarDetected = false;


	//allow us to debug see the image as bgr
	cv::Mat lineOutput;
    cv::cvtColor(frame, lineOutput, CV_GRAY2BGR);
///*
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> h;
    cv::findContours(frame, contours, h, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));


    for(size_t i=0; i<contours.size(); i++) {
        std::vector<cv::Point> c = contours[i]; //curent contour

		cv::RotatedRect rRect = cv::minAreaRect(c);
		std::vector<cv::Point> points;
		cv::Point2f vertices[4];
		rRect.points(vertices);
		rRect.angle = std::abs(rRect.angle);
		//debug view
		for (int i = 0; i < 4; i++) {
        	cv::line(lineOutput, vertices[i], vertices[(i+1)%4], cv::Scalar(0,0,255), 2);
		}
		cv::putText(lineOutput, std::to_string(rRect.angle), rRect.center, cv::FONT_HERSHEY_PLAIN, 1,  cv::Scalar(0,255,0, 255), 1);

		//these determine what makes a stop bar //#TODO: launch file params, convert using pixel_per_meter?
		double minAngleDegrees = 80;
		double maxAngleDegrees = 100;
		double minWidth = 2;
		double minHeight = 1;
		double triggerDistance = 1;

		double distance = frame.rows - rRect.center.y;

		double width;
		double height;
		if (rRect.size.width > rRect.size.height) {
			width = rRect.size.width;
			height = rRect.size.height;
		} else {
			width = rRect.size.height;
			height = rRect.size.width;
		}
		if (rRect.angle >= minAngleDegrees &&
			rRect.angle <= maxAngleDegrees &&
			width >= minWidth &&
			height >= minHeight &&
			distance <= triggerDistance) {

			stopBarDetected = true;

			ROS_INFO_STREAM("STOP BAR DETECTED");
			cv::putText(lineOutput, "AHHHHHHHHHHH!", rRect.center, cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,255,0, 255), 2);

		}


		/*
        cv::Vec4f line;
		cv::fitLine(c, line, cv::DIST_L1, 1, 0.01, 0.01);
		double vx = line[0];
		double vy = line[1];
		double x = line[2];
		double y = line[3];
		int lefty = (-x*vy/vx) + y;
		int righty = ((lineOutput.cols-x)*vy/vx) + y;
		cv::line(lineOutput, cv::Point(lineOutput.cols-1, righty), cv::Point(0, lefty), cv::Scalar(0,0,255), 2);
		*/
	}
//*/
/*
	ros::Time start = ros::Time::now();
    cv::Canny(frame, frame, 100, 100 * 3);

    // Standard Hough Line Transform
    std::vector<cv::Vec2f> lines; // will hold the results of the detection
    cv::HoughLines(frame, lines, 1, CV_PI/180, 50, 0, 0 ); // runs the actual detection
    // Draw the lines
    //ROS_INFO_STREAM(lines.size());
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
		double degreeMax = 95;
		double degreeMin = 85;
        if (theta >= degreeMin * CV_PI/180 && theta <= degreeMax * CV_PI/180) {
            cv::Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            cv::line( lineOutput, pt1, pt2, cv::Scalar(0,0,255), 3, cv::LINE_AA);
        }
    }
		ROS_INFO_STREAM(ros::Time::now() - start);
*/
    if (pubLine.getNumSubscribers() > 0) {
        sensor_msgs::Image outmsg;
        cv_ptr->image = lineOutput;
        cv_ptr->encoding = "bgr8";//"mono8";
        cv_ptr->toImageMsg(outmsg);
        pubLine.publish(outmsg);
    }

	if (stopBarDetected) { //only say the sign if we se the line!
		std_msgs::String moveMsg;
		moveMsg.data = currentMove;
		pubMove.publish(moveMsg);
	}
}


//loads images, scales as need be, and makes the rotated versions
void loadSignImages(std::string packageName, std::string fileName) {
  //#TODO: ADD SOME ERROR IN LOADING DETECTION!
  std::string path = ros::package::getPath(packageName);
  sign_forward = cv::imread(path + fileName);
  cv::Mat arrowEdges;
  cv::cvtColor(sign_forward, sign_forward, CV_RGB2GRAY);
  cv::resize(sign_forward, sign_forward, cv::Size(sign_forward.rows/6,sign_forward.cols/6)); //scale image down to make contour similar scale
  cv::Canny(sign_forward, arrowEdges, 100, 100*3 ); //get the edges as a binary, thresholds don't matter here

  //#TODO: LOAD THE ROTATED CONTOURS?
  std::vector<std::vector<cv::Point>> cnts;
  std::vector<cv::Vec4i> h;
  cv::findContours(arrowEdges, cnts, h, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
  template_contour_upright = cnts[0];

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
    nhp.param("roi_width", roi_width, -1); //-1 will default to the whole image
    nhp.param("roi_height", roi_height, -1);
    nhp.param("img_subscription", image_sub, std::string("/camera/image_color_rect"));
    nhp.param("sign_file_package_name", sign_file_package_name, std::string("rr_iarrc"));
    nhp.param("sign_file_path_from_package", sign_file_path_from_package, std::string("/src/sign_detector/sign_forward.jpg"));
    nhp.param("minimum_contour_area", minContourArea, 300.0);
    nhp.param("straight_match_similarity_threshold", straightMatchSimilarityThreshold, 0.3);
    nhp.param("turn_match_similarity_threshold", turnMatchSimilarityThreshold, 0.3);
    nhp.param("canny_threshold_low", cannyThresholdLow, 100.0);
    nhp.param("canny_threshold_high", cannyThresholdHigh, 100.0 * 3);

    loadSignImages(sign_file_package_name, sign_file_path_from_package);


    pub = nh.advertise<sensor_msgs::Image>("/signs_detected", 1); //debug publish of image
    pubLine = nh.advertise<sensor_msgs::Image>("/stop_bar_lines", 1); //debug publish of image
	pubMove = nh.advertise<std_msgs::String>("/turn_detected", 1); //publish the turn move for Urban Challenge
    auto img_real = nh.subscribe(image_sub, 1, sign_callback);
    auto stopBar = nh.subscribe("/lines_detection_img_transformed", 1, stopBar_callback);

    ros::spin();
    return 0;
}
