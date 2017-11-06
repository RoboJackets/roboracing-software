/*
 * Borrows from old color_detector_avc and from image_transform.
 * N.B. pass in a binary image in 1-channel "mono8" format.
 * Any non-zero element in the input image becomes a point in a
 * cloud.
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace cv;
using namespace ros;

using uchar = unsigned char;

map<string, Publisher> cloud_pubs;
double pxPerMeter;

void transformedImageCB(const sensor_msgs::ImageConstPtr& msg, string topic) {
    if(cloud_pubs[topic].getNumSubscribers() == 0) {
        // no one is listening. go home and cry
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch(cv_bridge::Exception& e) {
        ROS_ERROR("CV_Bridge error: %s", e.what());
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Mat transformed = cv_ptr->image;
    for(int r = 0; r < transformed.rows; r++) {
        uchar* row = transformed.ptr<uchar>(r);
        for(int c = 0; c < transformed.cols; c++) {
            if(row[3*c] || row[3*c + 1] || row[3*c + 2]) {

                pcl::PointXYZ point;
                point.y = -1 * (c - transformed.cols / 2.0f) / pxPerMeter;
                point.x = (transformed.rows - r) / pxPerMeter;
                point.z = 0.0;

                cloud->push_back(point);
            }
        }
    }
    pcl::PCLPointCloud2 cloud_pc2;
    pcl::toPCLPointCloud2(*cloud, cloud_pc2);
    sensor_msgs::PointCloud2 cloud_msg;
    pcl_conversions::fromPCL(cloud_pc2, cloud_msg);
    cloud_msg.header.frame_id = "base_footprint";
    cloud_msg.header.stamp = ros::Time::now();
    cloud_pubs[topic].publish(cloud_msg);
}

int main(int argc, char** argv) {
    init(argc, argv, "image_pcl_converter");
    NodeHandle nh;
    NodeHandle nhp("~");

    string topicsConcat;
    nhp.getParam("image_topics", topicsConcat);
    nhp.getParam("px_per_meter", pxPerMeter);

    vector<string> topics;
    boost::split(topics, topicsConcat, boost::is_any_of(" ,"));
    vector<Subscriber> image_subs;
    for(const string &topic : topics) {
        if(topic.size() == 0) continue;

        auto bound_callback = boost::bind(transformedImageCB, _1, topic);
        auto sub = nh.subscribe<sensor_msgs::Image>(topic, 1, bound_callback);
        image_subs.push_back(sub);

        ROS_INFO_STREAM("image_pcl_converter subscribed to " << topic);
        string newTopic = topic + "_cloud";
        ROS_INFO_STREAM("Creating new topic " << newTopic);
        cloud_pubs[topic] = nh.advertise<sensor_msgs::PointCloud2>(newTopic, 1);
    }

    spin();
    return 0;
}
