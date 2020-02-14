#include <ros/publisher.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <rr_evgp/UniformCostSearch.h>

using namespace std;

ros::Publisher pub_path;

/*
nav_msgs::Path convertPath(vector<Point> pointPath) {
    nav_msgs::Path pathMsg;
    for (Point p : pointPath) { //#TODO stamps and orientation
        geometry_msgs::PoseStamped poseStamped;
        //poseStamped.header.stamp =
        //{x,y,z} in real map
        poseStamped.pose.position.x = p.x; //#TODO: convert to real worl meters map
        poseStamped.pose.position.y = p.y;
        pathMsg.poses.push_back(poseStamped);
    }
    return pathMsg;
}
*/

/*
void map_callback(const nav_msgs::OccupancyGrid& msg) {
    //make 1d array accessible
    //distanceMap = msg->data; //#TODO: handle
    //make 1d array into 2d vector for ease of use right now #TODO: can we not?
    distanceMapVector.resize(msg.info.width);
    for (int i = 0; i < msg.info.width; i++) {
        distanceMapVector[i].resize(msg.info.height);
        for (int j = 0; j < msg.info.height; j++) {
            distanceMapVector[i][j] = msg.data[(j * msg.info.height) + i]; //#TODO: check if out of bounds
        }
    }

    vector<Point> pointPath = uniformCostSearch();
    nav_msgs::Path pathMsg = convertPath(pointPath);

    //#TODO: header and stuff
    pub_path.publish(pathMsg);
}
*/

void displayPathAsImage(std::vector<cv::Point> path, cv::Mat obstacleGrid) {
    cv::Mat displayImage;
    cv::cvtColor(obstacleGrid, displayImage, cv::COLOR_GRAY2BGR);
    for (cv::Point pt : path) {
        cv::Vec3b color(0,0,255);
        displayImage.at<cv::Vec3b>(pt) = color;
    }
    //cv::polylines(displayImage,path, false, cv::Scalar(0,0,255));
    cv::imshow("Path window", displayImage);
    cv::waitKey(0);
    //cv::imwrite("/home/brian/catkin_ws/src/roboracing-software/rr_evgp/src/center_path_finder/map6_smooth_pathBL.png")

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "center_path_finder");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string distance_map_sub_node;
    std::string distance_map_info_sub_node;
    nhp.param("distance_map_sub", distance_map_sub_node,
              string("/basic_mapper/costmap/costmap"));

    pub_path = nh.advertise<nav_msgs::Path>("/center_path", 1);
    //auto distance_map_sub = nh.subscribe(distance_map_sub_node, 1, map_callback);


    //ros::spin();

    //##for testing##
    ROS_INFO_STREAM("RUNNING");
    cv::Mat obstacleGrid;
    cv::Point startPt;
    cv::Point goalPt;

///*
    //Im5 map6_smooth
    obstacleGrid = cv::imread("/home/brian/catkin_ws/src/roboracing-software/rr_evgp/src/center_path_finder/map6_smooth.png", cv::IMREAD_GRAYSCALE);
    startPt.x = 38;
    startPt.y = 10;
    goalPt.x = 28;
    goalPt.y = 10;
//*/
/*
    //im1
    obstacleGrid = cv::imread("/home/brian/catkin_ws/src/roboracing-software/rr_evgp/src/center_path_finder/obstacleMap.png", cv::IMREAD_GRAYSCALE);
    startPt.x = 0;
    startPt.y = 0;
    goalPt.x = 0;
    goalPt.y = 3;
*/
/*
    //Im5 map6_smooth
    obstacleGrid = cv::imread("/home/brian/catkin_ws/src/roboracing-software/rr_evgp/src/center_path_finder/map3_large.png", cv::IMREAD_GRAYSCALE);
    startPt.x = 1584;
    startPt.y = 600;
    goalPt.x = 1096;
    goalPt.y = 600;
*/

///*
    //Im6 map7
    obstacleGrid = cv::imread("/home/brian/catkin_ws/src/roboracing-software/rr_evgp/src/center_path_finder/map7.png", cv::IMREAD_GRAYSCALE);
    startPt.x = 200;
    startPt.y = 55;
    goalPt.x = 140;
    goalPt.y = 55;
//*/



    cv::Mat distanceGrid;//    cv::resize(obstacleGrid, obstacleGrid, cv::Size(1000,1000));

    cv::distanceTransform(obstacleGrid, distanceGrid, cv::DIST_L2, cv::DIST_MASK_3);

    //convert based on max so that we minimize the center path
    double min;
    double max;
    cv::minMaxLoc(distanceGrid, &min, &max);
    cv::absdiff(distanceGrid, cv::Scalar(max), distanceGrid);


    cv::Mat temp;
    cv::normalize(distanceGrid, temp, 0.0, 1.0, cv::NORM_MINMAX); //visualize
    cv::imshow("window", temp);
    cv::waitKey(0);

    cv::bitwise_not(obstacleGrid, obstacleGrid); //black: clear; else: obstacle
    UniformCostSearch ucs(obstacleGrid, distanceGrid, startPt, goalPt);
    std::vector<cv::Point> rawPointPath = ucs.search();
    //ROS_INFO_STREAM(rawPointPath);

    displayPathAsImage(rawPointPath, obstacleGrid);


    return 0;
}
