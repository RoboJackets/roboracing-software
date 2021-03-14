//
// Created by Charles Jenkins on 11/27/20.
//

#include "tag_detection.h"

#include <utility>

tag_detection::tag_detection(ros::NodeHandle *nh, const std::string &camera_frame, const std::string &pointcloud,
                             const std::string &tag_detections_topic, const std::string &destination_frame,
                             const std::string &tag_detection_markers, double x_offset, double y_offset,
                             double px_per_m, double width, double height, std::vector<april_robot> robots) {
    this->camera_frame = camera_frame;
    this->destination_frame = destination_frame;
    this->width = width;
    this->robots = std::move(robots);

    double left = x_offset - width / 2;
    double bottom = y_offset;
    for (double x = 0; x < width; x += 1 / px_per_m) {
        for (double y = 0; y < height; y += 1 / px_per_m) {
            geometry_msgs::Point point;
            point.x = left + x;
            point.y = bottom + y;
            point.z = 0;

            pcl_outline.push_back(pcl::PointXYZ(point.x, point.y, 0));
            marker_outline.push_back(point);
        }
    }

    previous_outline = std::vector<pcl::PointCloud<pcl::PointXYZ>>(6);
    previous_marker = std::vector<visualization_msgs::Marker>(6);

    // Storing as an instance variable keeps subscriber alive
    sub_detections = nh->subscribe(tag_detections_topic, 1, &tag_detection::callback, this);
    pub_pointcloud = nh->advertise<sensor_msgs::PointCloud2>(pointcloud, 1);
    pub_markers = nh->advertise<visualization_msgs::Marker>(tag_detection_markers, 0);
}

void tag_detection::callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
    opponent_cloud.clear();
    auto msgs = msg->detections;
    std::vector<std::vector<std::pair<int, geometry_msgs::Pose>>> tag_groups(6);
    for (const auto &message : msgs) {        // Iterate through all discovered April Tags
        int car_number = message.id[0] / 10;  // By the specs, car 1 has tags 10,11,12... car 2 has 20,21,22...
        tag_groups[car_number].push_back(std::pair(message.id[0], message.pose.pose.pose));
        //        message.id[0]
        //        draw_opponent(message.id[0], message.pose.pose.pose);
    }
    draw_opponents(&tag_groups);
    publishPointCloud(opponent_cloud);
}

void tag_detection::publishPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud) {
    sensor_msgs::PointCloud2 outmsg;
    pcl::toROSMsg(cloud, outmsg);
    outmsg.header.frame_id = destination_frame;
    pub_pointcloud.publish(outmsg);
}

void tag_detection::draw_opponents(std::vector<std::vector<std::pair<int, geometry_msgs::Pose>>> *real_tags) {
    // w := our base footprint (world), o := optical camera, a := april image, l := april link,
    // b := their base footprint
    // x_T_y := Transform from y to x

    tf::StampedTransform w_T_o;
    tf_listener.lookupTransform(this->destination_frame, this->camera_frame, ros::Time(0), w_T_o);
    for (const auto &robot : *real_tags) {
        if (!robot.empty()) {
            tf::Pose robot_pose;
            std::vector<tf::Pose> robot_pose_vector(robot.size());
            int num = robot[0].first / 10;
            ROS_INFO_STREAM(" 3");
            int index = 0;
            for (const auto &tag : robot) {
                tf::Pose o_T_a;
                tf::poseMsgToTF(tag.second, o_T_a);

                tf::Transform a_T_l;
                a_T_l.setRotation(tf::createQuaternionFromRPY(0, M_PI / 2, M_PI / 2));
                // April tag has different rotation from april link

                // Target frame is rightmost digit + _april
                // eg: April tag id = 34, target frame is "april_4"
                std::string april_link = "april_" + std::to_string(tag.first % 10);
                tf::StampedTransform l_T_b;

                // Use the rear april tag to get the back of the robot
                // For an easy way to get the bounding box
                // An alternative method would be to base it off of the base footprint
                // and then shift backwards, but this is simpler
                tf_listener.lookupTransform(april_link, rear_april_tag, ros::Time(0), l_T_b);

                // Update location
                tf::Pose p_w = w_T_o * o_T_a * a_T_l * l_T_b;

                // To make your box outline point the right direction,
                // Not needed if you make the outline different dimension
                tf::Transform random_T;
                random_T.setRotation(tf::createQuaternionFromRPY(0, 0, -M_PI / 2));

                robot_pose = p_w * random_T;
                robot_pose_vector[index] = robot_pose;

                ROS_INFO_STREAM(april_link << " " << robot_pose.getOrigin().getX()
                                           << " " << robot_pose.getOrigin().getY() << " "
                                           << robot_pose.getOrigin().getZ());

//            // Just to viz points for debugging
//            visualization_msgs::Marker marker;
//            marker.header.frame_id = "base_footprint";
//            marker.header.stamp = ros::Time();
//            marker.ns = "";
//            marker.id = 1;
//            marker.type = visualization_msgs::Marker::SPHERE;
//            marker.action = visualization_msgs::Marker::ADD;
//            marker.pose.position.x = robot_pose.getOrigin().getX();
//            marker.pose.position.y = robot_pose.getOrigin().getY();
//            marker.pose.position.z = 0;
//            marker.pose.orientation.x = 0.0;
//            marker.pose.orientation.y = 0.0;
//            marker.pose.orientation.z = 0.0;
//            marker.pose.orientation.w = 1.0;
//            marker.scale.x = 0.1;
//            marker.scale.y = 0.1;
//            marker.scale.z = 0.1;
//            marker.color.a = 1.0;
//            marker.color.g = 1.0;
//            pub_markers.publish(marker);
                // Update point cloud for the tag
                pcl::PointCloud<pcl::PointXYZ> car_outline;
                Eigen::Affine3d affine3d;
                tf::transformTFToEigen(robot_pose, affine3d);
                pcl::transformPointCloud(pcl_outline, car_outline, affine3d);

                index++;
            }

            tf::Pose robot_pose_av = poseAverage(robot_pose_vector);
            if (robot_pose_av.getOrigin().getX() != 0) {
                ROS_INFO_STREAM(
                        "Average" << " " << robot_pose_av.getOrigin().getX() << " " << robot_pose_av.getOrigin().getY()
                                  << " " << robot_pose_av.getOrigin().getZ());
                visualization_msgs::Marker marker;
                marker.header.frame_id = "april_4";
                marker.header.stamp = ros::Time();
                marker.ns = "";
                marker.id = 2;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = robot_pose_av.getOrigin().getX();
                marker.pose.position.y = robot_pose_av.getOrigin().getY();
                marker.pose.position.z = 0;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.a = 1.0;
                marker.color.b = 1.0;
                pub_markers.publish(marker);

                pcl::PointCloud<pcl::PointXYZ> car_outline;
                Eigen::Affine3d affine3d;
                tf::transformTFToEigen(robot_pose_av, affine3d);
                pcl::transformPointCloud(pcl_outline, car_outline, affine3d);
                opponent_cloud += car_outline;
                previous_marker[num] = marker;
                previous_outline[num] = car_outline;
            } else {
                pub_markers.publish(previous_marker[num]);
                opponent_cloud = previous_outline[num];
            }
        }
    }
}

tf::Pose tag_detection::poseAverage(std::vector<tf::Pose> poses) {
    if (poses.size() > 1) {
        // throw out outliers
        std::set<tf::Pose> visited;
        std::vector<tf::Pose> pose_no_outliers;
        for (int i = 0; i < poses.size() && pose_no_outliers.size() < poses.size(); i++) {
            for (int j = 0; j < poses.size() && pose_no_outliers.size() < poses.size(); j++) {
                if (j != i) {
                    double distance = abs(poses[i].getOrigin().getX() - poses[j].getOrigin().getX());
                    distance += abs(poses[i].getOrigin().getY() - poses[j].getOrigin().getY());
                    ROS_INFO_STREAM("distance" << distance << " First " << poses[i].getOrigin().getX() << " Second "
                                               << poses[j].getOrigin().getX());
                    // Weight the results that have multiple clusters heavier
                    // by including them even if they have already been expanded
                    if (distance <= 1) {
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
            result.setRotation(pose_no_outliers[0].getRotation());

            for (tf::Pose pose : pose_no_outliers) {
                ROS_INFO_STREAM("out" << " " << pose.getOrigin().getX() << " " << pose.getOrigin().getY()
                                       << " " << pose.getOrigin().getZ());
                result.getOrigin().setX(result.getOrigin().getX() + pose.getOrigin().getX() / poses.size());
                result.getOrigin().setY(result.getOrigin().getY() + pose.getOrigin().getY() / poses.size());
            }

            //just take the orientation of one of them, no averaging here
            ROS_INFO_STREAM("Relt" << " " << result.getOrigin().getX() << " " << result.getOrigin().getY()
                                   << " " << result.getOrigin().getZ());
            return result;
        } else {
            return poses[0];
        }

        //        tf::Vector3 averagedVectors;
        //        tf::Quaternion averagedQuaternions;
        //        Eigen::Matrix4Xf quaternionMatrix;
        //
        //        for (int i = 0; i < poses.size(); i++) {
        //            averagedVectors += poses[i].getOrigin();
        //            quaternionMatrix(0, i) = poses[i].getRotation().getW();
        //            quaternionMatrix(1, i) = poses[i].getRotation().getX();
        //            quaternionMatrix(2, i) = poses[i].getRotation().getY();
        //            quaternionMatrix(3, i) = poses[i].getRotation().getZ();
        //        }
        //        Eigen::EigenSolver<Eigen::Matrix4Xf> solver(quaternionMatrix, true);
        return poses[0];
    } else if (poses.size() == 1) {
        return poses[0];
    }
    return tf::Pose::getIdentity();
}


