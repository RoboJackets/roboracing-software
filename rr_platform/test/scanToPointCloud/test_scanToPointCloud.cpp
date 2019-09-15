#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

class ScanToPointCloudTestSuite : public testing::Test {
  public:
    ScanToPointCloudTestSuite()
        : handle()
        , scan_pub(handle.advertise<sensor_msgs::LaserScan>("scan", 1))
        , pc_sub(handle.subscribe("scan/pointcloud", 1, &ScanToPointCloudTestSuite::cloudCallback, this)) {}

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
        cloud_msg = *msg;
        cloud_received = true;
    }

  protected:
    void SetUp() override {
        while (!IsNodeReady()) {
            ros::spinOnce();
        }
    }

    void TearDown() override {}

    bool IsNodeReady() {
        return (scan_pub.getNumSubscribers() > 0) && (pc_sub.getNumPublishers() > 0);
    }

    ros::NodeHandle handle;
    ros::Publisher scan_pub;
    ros::Subscriber pc_sub;
    volatile bool cloud_received = false;
    sensor_msgs::PointCloud2 cloud_msg;
};

TEST_F(ScanToPointCloudTestSuite, EmptyScan) {
    // Empty scans should result in empty PointClouds

    cloud_received = false;

    sensor_msgs::LaserScan scan;
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "/laser";
    scan.angle_min = -0.785398f;
    scan.angle_max = 3.92699f;
    scan.angle_increment = 0.0174533f;
    scan.time_increment = 0.0001852f;
    scan.scan_time = 1.0f / 15.0f;  // 15 Hz.
    scan.range_min = 0.05f;
    scan.range_max = 10.0f;
    scan.ranges = {};
    scan.intensities = {};

    scan_pub.publish(scan);

    while (!cloud_received) {
        ros::spinOnce();
    }

    EXPECT_TRUE(cloud_msg.data.empty());
}

TEST_F(ScanToPointCloudTestSuite, RemoveClosePoints) {
    // Points less than 1m from LIDAR should be filtered out

    cloud_received = false;

    sensor_msgs::LaserScan scan;
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "/laser";
    scan.angle_min = -0.785398f;
    scan.angle_max = 3.92699f;
    scan.angle_increment = 0.0174533f;
    scan.time_increment = 0.0001852f;
    scan.scan_time = 1.0f / 15.0f;  // 15 Hz.
    scan.range_min = 0.05f;
    scan.range_max = 10.0f;
    scan.ranges = { 0.5 };
    scan.intensities = {};

    scan_pub.publish(scan);

    while (!cloud_received) {
        ros::spinOnce();
    }

    EXPECT_TRUE(cloud_msg.data.empty());
}

TEST_F(ScanToPointCloudTestSuite, NormalScan) {
    // Normal points should be converted to cartesian coordinates correctly

    cloud_received = false;

    sensor_msgs::LaserScan scan;
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "/laser";
    scan.angle_min = -0.785398f;
    scan.angle_max = 3.92699f;
    scan.angle_increment = 0.0174533f;
    scan.time_increment = 0.0001852f;
    scan.scan_time = 1.0f / 15.0f;  // 15 Hz.
    scan.range_min = 0.05f;
    scan.range_max = 10.0f;
    scan.ranges.resize(271);
    std::fill(scan.ranges.begin(), scan.ranges.end(), 0.0f);
    scan.ranges[0] = 1.414213f;
    scan.ranges[45] = 1.0f;
    scan.ranges[90] = 1.414213;
    scan.ranges[135] = 1.0f;
    scan.ranges[180] = 1.414213f;
    scan.ranges[225] = 1.0f;
    scan.ranges[270] = 1.414213f;
    scan.intensities = {};

    scan_pub.publish(scan);

    while (!cloud_received) {
        ros::spinOnce();
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(cloud_msg, cloud);

    EXPECT_EQ(7, cloud.size());

    EXPECT_NEAR(1.0, cloud[0].x, 0.1);
    EXPECT_NEAR(-1.0, cloud[0].y, 0.1);

    EXPECT_NEAR(1.0, cloud[1].x, 0.1);
    EXPECT_NEAR(0.0, cloud[1].y, 0.1);

    EXPECT_NEAR(1.0, cloud[2].x, 0.1);
    EXPECT_NEAR(1.0, cloud[2].y, 0.1);

    EXPECT_NEAR(0.0, cloud[3].x, 0.1);
    EXPECT_NEAR(1.0, cloud[3].y, 0.1);

    EXPECT_NEAR(-1.0, cloud[4].x, 0.1);
    EXPECT_NEAR(1.0, cloud[4].y, 0.1);

    EXPECT_NEAR(-1.0, cloud[5].x, 0.1);
    EXPECT_NEAR(0.0, cloud[5].y, 0.1);

    EXPECT_NEAR(-1.0, cloud[6].x, 0.1);
    EXPECT_NEAR(-1.0, cloud[6].y, 0.1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_scanToPointCloud");
    testing::InitGoogleTest(&argc, argv);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}
