//
// Created by Charles Jenkins on 11/27/20.
//

#include "ground_segmentation.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <utility>
#include <iostream>

#include "ground_segmenter/ground_segmenter.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "ground_segmentation");
    ros::NodeHandle nhp("~");
    ros::NodeHandle nh;

    GroundSegmenterParams params = GroundSegmenterParams{};
    nhp.getParam("r_min_square", params.r_min_square);
    nhp.getParam("r_max_square", params.r_max_square);
    nhp.getParam("n_bins", params.n_bins);
    nhp.getParam("n_segments", params.n_segments);

    nhp.getParam("max_dist_to_line", params.max_dist_to_line);
    nhp.getParam("min_slope", params.min_slope);
    nhp.getParam("max_slope", params.max_slope);
    nhp.getParam("max_error_square", params.max_error_square);
    nhp.getParam("long_threshold", params.long_threshold);
    nhp.getParam("max_long_height", params.max_long_height);
    nhp.getParam("max_start_height", params.max_start_height);
    nhp.getParam("sensor_height", params.sensor_height);
    nhp.getParam("line_search_angle", params.line_search_angle);

    nhp.getParam("n_threads", params.n_threads);

    ground_segmentation ground_segmentation(&nh, params);

    ros::spin();
    return 0;
}

ground_segmentation::ground_segmentation(ros::NodeHandle *nh, GroundSegmenterParams params) {
    // Storing as an instance variable keeps subscriber alive
    velodyne_sub_ = nh->subscribe("/velodyne_points", 1, &ground_segmentation::callback, this);
    pcl_ground_pub_ = nh->advertise<sensor_msgs::PointCloud2>("/ground_segmentation/ground", 1);
    pcl_obstacle_pub_ = nh->advertise<sensor_msgs::PointCloud2>("/ground_segmentation/obstacle", 1);
    //pub_markers = nh->advertise<visualization_msgs::MarkerArray>(tag_detection_markers, 1);
    //pub_opponents = nh->advertise<geometry_msgs::PoseArray>(opponents_april, 1);

    nh->getParam("robot_width", robot_width);
    nh->getParam("robot_depth_forward", robot_depth_forward);
    nh->getParam("robot_depth_backward", robot_depth_backward);

    segmentation_params = params;
}

void ground_segmentation::callback(const sensor_msgs::PointCloud2 &cloud) {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud{};
    pcl::fromROSMsg(cloud, pcl_cloud);

    GroundSegmenter segmenter(segmentation_params);
    pcl::PointCloud<pcl::PointXYZ> cloud_transformed;

    std::string frame_id = cloud.header.frame_id;

    std::vector<int> labels;

    // todo: transform cloud so that the vertical axis (z) is always up relative to gravity
    // Eigen::Affine3d tf = Eigen::Affine3d::Identity<double, 3, 2>;
    // pcl::transformPointCloud(pcl_cloud, cloud_transformed, tf);

    for (size_t i = 0; i < pcl_cloud.size(); ++i) {
        auto pt = pcl_cloud[i];
        if (fabs(pt.y) > robot_width) continue;
        if (!(robot_depth_backward < pt.x && pt.x < robot_depth_forward)) continue;

        cloud_transformed.push_back(pt);
    }

    segmenter.segment(cloud_transformed, &labels);
    pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud;

    ground_cloud.sensor_orientation_ = pcl_cloud.sensor_orientation_;
    ground_cloud.sensor_origin_ = pcl_cloud.sensor_origin_;

    obstacle_cloud.sensor_orientation_ = pcl_cloud.sensor_orientation_;
    obstacle_cloud.sensor_origin_ = pcl_cloud.sensor_origin_;

    for (size_t i = 0; i < cloud_transformed.size(); ++i) {
        if (labels[i] == 1) {
            ground_cloud.push_back(cloud_transformed[i]);
        } else {
            obstacle_cloud.push_back(cloud_transformed[i]);
        }
    }

    publishPointCloud(ground_cloud, pcl_ground_pub_, frame_id);
    publishPointCloud(obstacle_cloud, pcl_obstacle_pub_, frame_id);

    //pcl::PointCloud<pcl::PointXYZ> cloud2 = pcl::PointCloud<pcl::PointXYZ>(cloud);
    //publishPointCloud(pcl_cloud, pcl_ground_pub_);
    
    //draw_opponents(&tag_groups);
}

void ground_segmentation::publishPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud, ros::Publisher &pub, std::string frame_id) {
    sensor_msgs::PointCloud2 outmsg;
    pcl::toROSMsg(cloud, outmsg);
    outmsg.header.frame_id = frame_id;
    pub.publish(outmsg);

    std::cout << "Published cloud with " << cloud.size() << " points to " << pub.getTopic() << std::endl;
}

tf::Pose ground_segmentation::poseAverage(std::vector<tf::Pose> poses) {
    if (poses.size() > 1) {
        // throw out outliers
        std::set<tf::Pose> visited;
        std::vector<tf::Pose> pose_no_outliers;
        for (int i = 0; i < poses.size() && pose_no_outliers.size() < poses.size(); i++) {
            for (int j = 0; j < poses.size() && pose_no_outliers.size() < poses.size(); j++) {
                if (j != i) {
                    double distance = abs(poses[i].getOrigin().getX() - poses[j].getOrigin().getX());
                    distance += abs(poses[i].getOrigin().getY() - poses[j].getOrigin().getY());
                    if (distance <= pose_distance) {
                        pose_no_outliers.push_back(poses[i]);
                        pose_no_outliers.push_back(poses[j]);
                    }
                }
            }
        }
        if (!pose_no_outliers.empty()) {
            tf::Pose result;
            result.getOrigin().setX(0);
            result.getOrigin().setY(0);
            result.getOrigin().setZ(0);

            // Just take orientation of one of them
            std::vector<tf::Quaternion> quaternions;

            for (tf::Pose pose : pose_no_outliers) {
                tf::Quaternion qt;
                qt = pose.getRotation();
                tf::Matrix3x3 mat(qt);
                double roll, pitch, yaw;
                mat.getRPY(roll, pitch, yaw);
                qt.setRPY(0, 0, yaw);  // disregard rotation up and down since car should be flat on the ground!
                quaternions.push_back(qt);
                result.getOrigin().setX(result.getOrigin().getX() + pose.getOrigin().getX() / pose_no_outliers.size());
                result.getOrigin().setY(result.getOrigin().getY() + pose.getOrigin().getY() / pose_no_outliers.size());
            }

            std::vector<double> weights(quaternions.size(), 1);
            tf::Quaternion av_rot = getAverageQuaternion(quaternions, weights);
            result.setRotation(av_rot);

            return result;
        } else {
            tf::Pose result;
            result.getOrigin().setX(poses[0].getOrigin().getX());
            result.getOrigin().setY(poses[0].getOrigin().getY());
            result.getOrigin().setZ(0);
            return result;
        }
    } else if (poses.size() == 1) {
        tf::Pose result;
        result.getOrigin().setX(poses[0].getOrigin().getX());
        result.getOrigin().setY(poses[0].getOrigin().getY());
        result.getOrigin().setZ(0);
        return result;
    }
    return tf::Pose::getIdentity();
}

tf::Quaternion ground_segmentation::getAverageQuaternion(std::vector<tf::Quaternion> &quaternions,
                                                   std::vector<double> &weights) {
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, quaternions.size());
    Eigen::Vector3d vec;
    for (size_t i = 0; i < quaternions.size(); ++i) {
        // Weigh the quaternions according to their associated weight
        tf::Quaternion quat = quaternions[i] * weights[i];
        // Append the weighted Quaternion to a matrix Q.
        Q(0, i) = quat.x();
        Q(1, i) = quat.y();
        Q(2, i) = quat.z();
        Q(3, i) = quat.w();
    }

    // Create a solver for finding the eigenvectors and eigenvalues
    Eigen::EigenSolver<Eigen::MatrixXd> es(Q * Q.transpose());

    // Find index of maximum (real) Eigenvalue.
    auto eigenvalues = es.eigenvalues();
    size_t max_idx = 0;
    double max_value = eigenvalues[max_idx].real();
    for (size_t i = 1; i < 4; ++i) {
        double real = eigenvalues[i].real();
        if (real > max_value) {
            max_value = real;
            max_idx = i;
        }
    }

    // Get corresponding Eigenvector, normalize it and return it as the average quat
    auto eigenvector = es.eigenvectors().col(max_idx).normalized();

    tf::Quaternion mean_orientation(eigenvector[0].real(), eigenvector[1].real(), eigenvector[2].real(),
                                    eigenvector[3].real());

    return mean_orientation;
}
