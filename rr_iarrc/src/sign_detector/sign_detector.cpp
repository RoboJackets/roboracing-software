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

std::vector<cv::Point> template_contour_upright;


void img_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;

    cv::Mat crop;
    if (roi_width == -1 || roi_height == -1) {
      crop = frame;
    } else {
      cv::Rect roi(roi_x, roi_y, roi_width, roi_height);
      crop = frame(roi); //note that crop is just a reference to that roi of the frame, not a copy
    }

std::string path1 = ros::package::getPath("rr_iarrc");
crop =  cv::imread(path1 + "/src/sign_detector/traffic_signs.png");

    cv::GaussianBlur(crop, crop, cv::Size(5,5), 0, 0, cv::BORDER_DEFAULT); //may or may not help Canny

    //Color -> binary; Edge detection
    cv::Mat edges;
    double thresh1 = 100; //TODO: make these launch params
    double thresh2 = thresh1 * 3;
    cv::Canny(crop, edges, thresh1, thresh2 );

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    cv::morphologyEx(edges, edges, cv::MORPH_CLOSE, kernel);

    //Find arrow-like shapes with Contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    std::vector<cv::Mat> channels(3);
    channels[0] = edges;
    channels[1] = edges;
    channels[2] = edges;
    cv::merge(channels, crop);


    //cv::drawContours(crop, contours, -1, cv::Scalar(0,255,0), 2); //debug #TODO: REMOVE

    for(size_t i=0; i<contours.size(); i++) {
      std::vector<cv::Point> c = contours[i]; //curent contour
      double perimeter = cv::arcLength(c, true);
      double epsilon = 0.04 * perimeter; //#TODO: PLAY WITH THIS AS IT DETERMINES HOW GOOD THE APPROXIMATION IS
      std::vector<cv::Point> approxC;
      cv::approxPolyDP(c, approxC, epsilon, true);
      if (approxC.size() >= 7 || approxC.size() <= 9) { //#TODO: adjust bounds of what is an arrow after adjusting epsilon
          //check the ratio is arrow-like
          cv::Rect rect = cv::boundingRect(c); //#TODO: maybe do the bounding rectc check before approximation
          double ratio = 1.5; //ratio of width or height or vice versa
          if (rect.width * ratio <= rect.height || rect.height * ratio <= rect.width) {
            cv::rectangle(crop, rect, cv::Scalar(0,255,255),3); //debug #TODO: REMOVE

            if (rect.width > rect.height) {
              //Sideways
              double matchThreshold = 0.2; //#TODO: play with this #
              double matchLikeness = cv::matchShapes(c, template_contour_upright, CV_CONTOURS_MATCH_I1, 0); //#TODO: change to sideways contours
              cv::putText(crop, std::to_string(matchLikeness), cv::Point(rect.x, rect.y +10), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255, 255), 2);

              if (matchLikeness < matchThreshold) {
                  // find top point of the arrow and test its x location
                  auto extremeY = std::minmax_element(c.begin(), c.end(), [](cv::Point const& a, cv::Point const& b){
                      return a.y < b.y;
                  });
                  if (extremeY.first->x < rect.x + rect.width/2) {//topmost point is far left
                    //left pointing arrow!
                    cv::putText(crop, "Left!", rect.tl(), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,255,0, 255), 2);
                    cv::putText(crop, std::to_string(matchLikeness), cv::Point(rect.x, rect.y +10), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255, 255), 2);
                  } else {
                    //right pointing arrow!
                    cv::putText(crop, "Right!", rect.tl(), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,255,0), 2);
                    cv::putText(crop, std::to_string(matchLikeness), cv::Point(rect.x, rect.y + 10), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255, 255), 2);

                  }

              }
            } else {
              //Straight
              double matchThreshold = 0.2; //#TODO: play with this #
              double matchLikeness = cv::matchShapes(c, template_contour_upright, CV_CONTOURS_MATCH_I2, 0.0);
              if (matchLikeness < matchThreshold) {
                  cv::putText(crop, "Forward!", rect.tl(), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,255,0), 2);
                  cv::putText(crop, std::to_string(matchLikeness), cv::Point(rect.x, rect.y + 10), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255, 255), 2);


              }
            }

        }
      }


    }

    //Some debug images for us
    //show where the match is found
    //cv::Mat sign = sign_forward;
    //cv::rectangle(crop, matchLoc, cv::Point(matchLoc.x + sign.cols, matchLoc.y + sign.rows), cv::Scalar(0, 255, 0), 2, 8, 0);
    //show where we are cropped to
    cv::rectangle(crop, cv::Point(0,0), cv::Point(crop.cols-1, crop.rows-1), cv::Scalar(0,255,0), 2, 8 ,0);

    if (pub.getNumSubscribers() > 0) {
        sensor_msgs::Image outmsg;
        cv_ptr->image = crop;//frame;
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
  cv::Mat arrowEdges;
  cv::cvtColor(sign_forward, sign_forward, CV_RGB2GRAY); //single channel helps with templateMatch
  cv::resize(sign_forward, sign_forward, cv::Size(sign_forward.rows/6,sign_forward.cols/6)); //scale image down to make contour similar scale
  cv::Canny(sign_forward, arrowEdges, 100, 100*3 ); //get the edges as a binary, thresholds don't matter
  //currently sign is 2000x2000px
  //cv::resize(sign_forward, sign_forward, cv::Size(300,300));//110,110)); //scale image down #TODO: more options

  //#TODO: LOAD THE CONTOURS
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
