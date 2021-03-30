//
// Created by Charles Jenkins on 11/27/20.
//

#include "tag_detection.h"

#include <utility>

int main(int argc, char **argv) {
    ros::init(argc, argv, "april_tag_pointcloud");
    ros::NodeHandle nhp("~");
    ros::NodeHandle nh;
    std::string camera_frame, pointcloud, tag_detections_topic, tag_detection_markers, destination_frame,
          opponents_april;
    double x_offset, y_offset, px_per_m, width, height;
    XmlRpc::XmlRpcValue tags;
    nhp.getParam("camera_frame", camera_frame);
    nhp.getParam("pointcloud", pointcloud);
    nhp.getParam("tag_detections_topic", tag_detections_topic);
    nhp.getParam("tag_detection_markers", tag_detection_markers);
    nhp.getParam("opponents_april", opponents_april);
    nhp.getParam("destination_frame", destination_frame);
    nhp.getParam("x_offset", x_offset);
    nhp.getParam("y_offset", y_offset);
    nhp.getParam("px_per_m", px_per_m);
    nhp.getParam("width", width);
    nhp.getParam("height", height);
    nhp.getParam("tags", tags);
    tag_detection tagDetection(&nh, camera_frame, pointcloud, tag_detections_topic, destination_frame,
                               tag_detection_markers, x_offset, y_offset, px_per_m, width, height, opponents_april);

    ros::spin();
    return 0;
}

tag_detection::tag_detection(ros::NodeHandle *nh, const std::string &camera_frame, const std::string &pointcloud,
                             const std::string &tag_detections_topic, const std::string &destination_frame,
                             const std::string &tag_detection_markers, double x_offset, double y_offset,
                             double px_per_m, double width, double height, const std::string &opponents_april) {
    this->camera_frame = camera_frame;
    this->destination_frame = destination_frame;
    this->width = width;

    // Create rectangle point cloud that will be centered around each car
    double left = x_offset - width / 2;
    double bottom = y_offset;
    for (double x = 0; x < width; x += 1 / px_per_m) {
        for (double y = 0; y < height; y += 1 / px_per_m) {
            geometry_msgs::Point point;
            point.x = left + x;
            point.y = bottom + y;
            point.z = 0;

            pcl_outline.push_back(pcl::PointXYZ(point.x, point.y, 0));
        }
    }

    // Storing as an instance variable keeps subscriber alive
    sub_detections = nh->subscribe(tag_detections_topic, 1, &tag_detection::callback, this);
    pub_pointcloud = nh->advertise<sensor_msgs::PointCloud2>(pointcloud, 1);
    pub_markers = nh->advertise<visualization_msgs::MarkerArray>(tag_detection_markers, 1);
    pub_opponents = nh->advertise<geometry_msgs::PoseArray>(opponents_april, 1);
}

void tag_detection::callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
    auto msgs = msg->detections;
    std::vector<std::vector<std::pair<int, geometry_msgs::Pose>>> tag_groups(5);
    for (const auto &message : msgs) {        // Iterate through all discovered April Tags
        int car_number = message.id[0] / 10;  // By the specs, car 1 has tags 10,11,12... car 2 has 20,21,22...
        tag_groups[car_number].push_back(std::pair(message.id[0], message.pose.pose.pose));
    }
    draw_opponents(&tag_groups);
}

void tag_detection::draw_opponents(std::vector<std::vector<std::pair<int, geometry_msgs::Pose>>> *real_tags) {
    // w := our base footprint (world), o := optical camera, a := april image, l := april link,
    // b := their base footprint
    // x_T_y := Transform from y to x
    geometry_msgs::PoseArray opponent_averages;
    pcl::PointCloud<pcl::PointXYZ> opponent_cloud;
    visualization_msgs::MarkerArray marker_array;
    tf::StampedTransform w_T_o;
    tf_listener.lookupTransform(this->destination_frame, this->camera_frame, ros::Time(0), w_T_o);
    for (const auto &robot : *real_tags) {
        if (!robot.empty()) {
            std::vector<tf::Pose> robot_pose_vector(robot.size());
            int robot_num = robot[0].first / 10;  // Get id of robot
            tf::Pose robot_pose;
            int index = 0;
            for (const auto &tag : robot) {
                tf::Pose o_T_a;
                tf::poseMsgToTF(tag.second, o_T_a);

                tf::Transform a_T_l;
                a_T_l.setRotation(tf::createQuaternionFromRPY(0, M_PI / 2, M_PI / 2));
                // April tag has different rotation from april link

                // Target frame is rightmost digit + _april
                // eg: April tag id = 34, target frame is "4_april"
                std::string april_link = "april_" + std::to_string(tag.first % 10);
                tf::StampedTransform l_T_b;

                // Use back of the car for a simple transform
                tf_listener.lookupTransform(april_link, rear_april_tag, ros::Time(0), l_T_b);

                // Update location
                tf::Pose p_w = w_T_o * o_T_a * a_T_l * l_T_b;

                // To make your box outline point the right direction,
                // Not needed if you make the outline different dimension
                tf::Transform random_T;
                random_T.setRotation(tf::createQuaternionFromRPY(0, 0, -M_PI / 2));

                robot_pose = p_w * random_T;
                robot_pose_vector[index] = robot_pose;

                // Just to viz points for debugging (without them, only one car is rendered also however)
                visualization_msgs::Marker marker;
                marker.header.frame_id = "april_4";
                marker.header.stamp = ros::Time();
                marker.ns = "";
                marker.id = 1;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = robot_pose_vector[index].getOrigin().getX();
                marker.pose.position.y = robot_pose_vector[index].getOrigin().getY();
                marker.pose.position.z = 0;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.a = 1.0;
                marker.color.g = 1.0;

                pcl::PointCloud<pcl::PointXYZ> car_outline;
                Eigen::Affine3d affine3d;
                tf::transformTFToEigen(robot_pose_vector[index], affine3d);
                pcl::transformPointCloud(pcl_outline, car_outline, affine3d);

                index++;
            }
            tf::Pose robot_pose_av = poseAverage(robot_pose_vector);
            geometry_msgs::Pose pose;
            tf::poseTFToMsg(robot_pose_av, pose);
            opponent_averages.poses.push_back(pose);

            pcl::PointCloud<pcl::PointXYZ> car_outline;
            Eigen::Affine3d affine3d;
            tf::transformTFToEigen(robot_pose_av, affine3d);
            pcl::transformPointCloud(pcl_outline, car_outline, affine3d);
            opponent_cloud += (car_outline);

            visualization_msgs::Marker marker;
            marker.header.frame_id = "base_footprint";
            marker.header.stamp = ros::Time();
            marker.ns = "";
            marker.id = robot_num;
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
            marker_array.markers.push_back(marker);
        }
    }
    publishPointCloud(opponent_cloud);
    pub_markers.publish(marker_array);
    pub_opponents.publish(opponent_averages);
}

void tag_detection::publishPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud) {
    sensor_msgs::PointCloud2 outmsg;
    pcl::toROSMsg(cloud, outmsg);
    outmsg.header.frame_id = destination_frame;
    pub_pointcloud.publish(outmsg);
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

tf::Quaternion tag_detection::getAverageQuaternion(std::vector<tf::Quaternion> &quaternions,
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
