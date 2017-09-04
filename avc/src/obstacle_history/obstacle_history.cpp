#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <rr_platform/chassis_state.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> cloud_t;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_t;

ros::Publisher resultPub;

vector<int> cloud_sizes_queue;
cloud_ptr_t cloud_ptr(new cloud_t);
float yaw = 0, lastYaw = 0;
float speed = 0;
double lastUpdateSeconds = 0;

cloud_t transformedCloud;

/*
 * useful methods
 *
 * void pcl_ros::transformPointCloud (const pcl::PointCloud< PointT > &cloud_in,
 *                              pcl::PointCloud< PointT > &cloud_out,
 *                              const tf::Transform &transform)
 *
 * iterator pcl::PointCloud<T>::erase(iterator first, iterator last)
 */

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg) {
    yaw = tf::getYaw(msg->orientation);
}

void chassis_state_callback(const rr_platform::chassis_state::ConstPtr &msg) {
    speed = msg->speed_mps;
}

void obstacles_callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
//    cout << "0" << endl;

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    cloud_ptr_t newCloudPtr(new cloud_t);
    pcl::fromPCLPointCloud2(pcl_pc2, *newCloudPtr);

//    cout << "1" << endl;
//    cout << "size 1: " << cloud_ptr->size() << endl;

    int nRemove = 0;
    if(cloud_sizes_queue.size() >= 25) {
        nRemove = cloud_sizes_queue[0];
        cloud_sizes_queue.erase(cloud_sizes_queue.begin());
    }
    cloud_sizes_queue.push_back(newCloudPtr->size());

    if(nRemove > 0) {
        cloud_ptr->erase(cloud_ptr->begin(), cloud_ptr->begin() + nRemove);
    }

//    cout << "2" << endl;
//    cout << "size 2: " << cloud_ptr->size() << endl;

    tf::Quaternion tfQuaternion;
    tfQuaternion.setRPY(0, 0, -(yaw - lastYaw));
//    cout << "yaw diff = " << yaw - lastYaw << endl;
    lastYaw = yaw;

    float estimatedDistance = speed * (ros::Time::now().toSec() - lastUpdateSeconds);
    lastUpdateSeconds = ros::Time::now().toSec();
//    cout << "estimatedDistance = " << estimatedDistance << endl;

    tf::Transform tfTransform;
    tfTransform.setRotation(tfQuaternion);
    tf::Vector3 offset(-estimatedDistance, 0, 0);
    tfTransform.setOrigin(offset);

//    cout << "3" << endl;


//    cout << "3.1" << endl;
//    pcl_ros::transformPointCloud(*cloud_ptr, transformedCloud, tfTransform);
    pcl_ros::transformPointCloud(*cloud_ptr, *cloud_ptr, tfTransform);
//    cout << "3.2" << endl;
//    cloud_ptr.swap(transformedCloudPtr);

//    cout << "4" << endl;

    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(0.3f, 0.3f, 0.3f);

    cout << "size before = " << cloud_ptr->size() << endl;
    cloud_ptr->insert(cloud_ptr->end(), newCloudPtr->begin(), newCloudPtr->end());
    cout << "size after =  " << cloud_ptr->size() << endl;

//    cout << "5" << endl;

    cloud_t filteredCloud;
    filter.setInputCloud(cloud_ptr);
    filter.filter(filteredCloud);
//    filter.filter(*cloud_ptr);
//    cloud_ptr.swap(filteredCloudPtr);

//    cout << "6" << endl;

    pcl::PCLPointCloud2 resultPC2;
    pcl::toPCLPointCloud2(filteredCloud, resultPC2);
    sensor_msgs::PointCloud2 resultMsg;
    pcl_conversions::fromPCL(resultPC2, resultMsg);

    resultMsg.header.frame_id = "base_footprint";
    resultMsg.header.stamp = ros::Time::now();

    resultPub.publish(resultMsg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_history");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    auto obstaclesSub = nh.subscribe("/current_obstacles", 1, obstacles_callback);
    auto imuSub = nh.subscribe("/imu", 1, imu_callback);
    auto chassisStateSub = nh.subscribe("/chassis_state", 1, chassis_state_callback);

    resultPub = nh.advertise<sensor_msgs::PointCloud2>("/map", 1);

    lastUpdateSeconds = ros::Time::now().toSec();

    ros::spin();
    return 0;
}