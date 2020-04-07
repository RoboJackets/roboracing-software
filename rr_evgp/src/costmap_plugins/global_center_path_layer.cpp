#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <dynamic_reconfigure/server.h>
#include <parameter_assertions/assertions.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <rr_evgp/UniformCostSearch.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Path.h>


/*
 * @author Brian Cochran
 * This layer finds the center line of the course
 * using UniformCostSearch with costs being the distance
 * to the nearest obstacle.
*/

namespace rr {

class GlobalCenterPathLayer : public costmap_2d::Layer {
  public:
    void onInitialize() override {
        dsrv_ = std::make_unique<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>>();
        dsrv_->setCallback(boost::bind(&GlobalCenterPathLayer::reconfigureCB, this, _1, _2));

        init_robot_x = 0.0;
        init_robot_y = 0.0;
        init_robot_yaw = 0.0;
        init_is_set = false;
        lap_completed = false;
        center_path_calculated = false;
        has_left_initial_region = false;


        ros::NodeHandle nh;
        ros::NodeHandle costmap_nh("~costmap");
        ros::NodeHandle private_nh("~" + getName());
        std::string pathOutputTopic;
        assertions::param(private_nh, "output_topic", pathOutputTopic, std::string("/global_center_path"));
        pub_path = nh.advertise<nav_msgs::Path>(pathOutputTopic, 1);

        assertions::getParam(private_nh, "wall_length", wall_length_, { assertions::greater<double>(0) });
        assertions::getParam(private_nh, "wall_distance_behind_start", wall_distance_behind_start_, { assertions::greater<double>(0) });
        assertions::getParam(private_nh, "wall_thickness", wall_thickness_, { assertions::greater<double>(0) });
        assertions::getParam(private_nh, "scaling_factor", scaling_factor_, { assertions::greater_eq<double>(1.0) });
        assertions::getParam(private_nh, "distance_from_start", distance_from_start_, { assertions::greater<double>(0) });
        assertions::getParam(private_nh, "goal_distance_behind_start", goal_distance_behind_start_, { assertions::greater<double>(wall_distance_behind_start_) });
        assertions::getParam(private_nh, "forward_initial_offset", forward_initial_offset_, { assertions::less<double>(distance_from_start_) });

        assertions::getParam(costmap_nh, "global_frame", global_frame_);

    }

    void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                      double* max_y) override {
        if (!enabled_) {
            return;
        }

        curr_robot_x = robot_x;
        curr_robot_y = robot_y;
        if (!init_is_set) {
            cv::Point initPos = offsetPoint(cv::Point(curr_robot_x, curr_robot_y), init_robot_yaw, forward_initial_offset_);
            init_robot_x = initPos.x;
            init_robot_y = initPos.y;
            init_robot_yaw = robot_yaw;
            init_is_set = true;
        }

    }

    void updateCosts(costmap_2d::Costmap2D& master_grid, int min_cell_x, int min_cell_y, int max_cell_x,
                     int max_cell_y) override {
        if (!enabled_) {
            return;
        }

        double dx = curr_robot_x - init_robot_x;
        double dy = curr_robot_y - init_robot_y;
        double current_distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)); //c^2 = a^2 + b^2

        if (current_distance > distance_from_start_ + 1.0) { //#TODO:should this extra buffer be a param?
            has_left_initial_region = true;
        }

        if (has_left_initial_region) {
            if (current_distance < distance_from_start_) {
                lap_completed = true;
            }
        }

        if (lap_completed) {

            if (!center_path_calculated) {
                center_path_calculated = true;
                ROS_INFO_STREAM("Center Line Layer: Calculating center path");

                //convert to opencv matrix
                cv::Mat mat_grid(master_grid.getSizeInCellsX(), master_grid.getSizeInCellsY(), CV_8UC1);
                uint8_t* charMap = master_grid.getCharMap();
                for (int r = 0; r < mat_grid.rows; r++) {
                    for (int c = 0; c < mat_grid.cols; c++) {
                        mat_grid.at<uint8_t>(r, c) = (charMap[r * mat_grid.cols + c] != costmap_2d::FREE_SPACE) ? UniformCostSearch::OBSTACLE_SPACE : UniformCostSearch::FREE_SPACE;
                    }
                }

                //draw a wall to block the backwards path
                unsigned int wallLengthPixels = master_grid.cellDistance(wall_length_);
                unsigned int wallDistanceBehindStartPixels = master_grid.cellDistance(wall_distance_behind_start_);
                unsigned int wallThicknessPixels = master_grid.cellDistance(wall_thickness_);
                int mx;
                int my;
                master_grid.worldToMapEnforceBounds(init_robot_x, init_robot_y, mx, my);
                cv::Point2d wallMidpoint = offsetPoint(cv::Point(mx, my), init_robot_yaw, -1.0 * wallDistanceBehindStartPixels / 2.0);;
                cv::Point2d wallPtA = offsetPoint(wallMidpoint, init_robot_yaw + CV_PI / 2.0, wallLengthPixels / 2.0);
                cv::Point2d wallPtB = offsetPoint(wallMidpoint, init_robot_yaw - CV_PI / 2.0, wallLengthPixels / 2.0);


                unsigned int goalDistanceBehindStartPixels = master_grid.cellDistance(goal_distance_behind_start_);
                cv::Point2d goalPt = offsetPoint(cv::Point(mx, my), init_robot_yaw, -1.0 * goalDistanceBehindStartPixels);

                int cx;
                int cy;
                master_grid.worldToMapEnforceBounds(curr_robot_x, curr_robot_y, cx, cy);

                cv::Mat debug;
                cv::cvtColor(mat_grid, debug, CV_GRAY2BGR);
                cv::line(debug, wallPtA, wallPtB, CV_RGB(255,0,0), wallThicknessPixels); //add wall in between start and end points

                cv::Mat channels[3];
                cv::split(debug, channels);

                cv::Mat obstacleGrid;
                cv::resize(channels[2], obstacleGrid, cv::Size(master_grid.getSizeInCellsX() / scaling_factor_, master_grid.getSizeInCellsY() / scaling_factor_));
                cv::threshold(obstacleGrid, obstacleGrid, 127, UniformCostSearch::OBSTACLE_SPACE, cv::THRESH_BINARY); //threshold after scaling because of interpolation (halfway at 127 seems fine)

                //Ensures track walls are at least 2 pixels wide (can't sneak through) and adds buffer in case of badly sensed wall
                cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
                cv::dilate(obstacleGrid, obstacleGrid, kernel);


                cv::bitwise_not(obstacleGrid, obstacleGrid); //Flip due to how DistanceTransform defines obstacles
                cv::Mat distanceGrid;
                cv::distanceTransform(obstacleGrid, distanceGrid, cv::DIST_L2, cv::DIST_MASK_3);
                //convert based on max so that we minimize the center path
                double min;
                double max;
                cv::minMaxLoc(distanceGrid, &min, &max);
                cv::absdiff(distanceGrid, cv::Scalar(max), distanceGrid);
                cv::bitwise_not(obstacleGrid, obstacleGrid);

                cv::Point startPt(mx / scaling_factor_, my / scaling_factor_);
                goalPt.x = goalPt.x / scaling_factor_;
                goalPt.y = goalPt.y / scaling_factor_;
                UniformCostSearch ucs(obstacleGrid, distanceGrid, startPt, goalPt);

                //In case start or goal end up in the wall accidentally, find a workable point
                cv::Point newStartPt = ucs.getNearestFreePointBFS(startPt);
                ucs.setGoalPoint(newStartPt);
                cv::Point newGoalPt = ucs.getNearestFreePointBFS(goalPt);
                ucs.setGoalPoint(newGoalPt);

                std::vector<cv::Point> pixelPointPath = ucs.search();

                pathMsg.header.stamp = ros::Time::now();
                pathMsg.header.frame_id = global_frame_;
                for (cv::Point p : pixelPointPath) {
                    geometry_msgs::PoseStamped poseStamped;
                    //{x,y,z} in real map

                    master_grid.mapToWorld(p.x * scaling_factor_, p.y * scaling_factor_, poseStamped.pose.position.x, poseStamped.pose.position.y);
                    pathMsg.poses.push_back(poseStamped);
                }
                if (pub_path.getNumSubscribers() > 0) {
                    pub_path.publish(pathMsg);
                }

            }
        }

        if (center_path_calculated) {
            if (pub_path.getNumSubscribers() > 0) {
                pub_path.publish(pathMsg);
            }
        }

    }

  private:
    void reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level) {
        enabled_ = config.enabled;
    }

    cv::Point offsetPoint(cv::Point startPoint, double angle, double distance) {
        cv::Point outputPoint;
        outputPoint.x = std::cos(angle) * distance + startPoint.x;
        outputPoint.y = std::sin(angle) * distance + startPoint.y;
        return outputPoint;
    }


    // state
    double init_robot_x;
    double init_robot_y;
    double init_robot_yaw;
    double curr_robot_x;
    double curr_robot_y;
    double wall_length_;
    double wall_thickness_;
    double wall_distance_behind_start_;
    double distance_from_start_;
    double forward_initial_offset_;
    double goal_distance_behind_start_;
    double scaling_factor_;
    std::string global_frame_;
    bool lap_completed;
    bool init_is_set;
    bool center_path_calculated;
    bool has_left_initial_region;
    ros::Publisher pub_path;
    nav_msgs::Path pathMsg;

    std::unique_ptr<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>> dsrv_;
};

}  // namespace rr

PLUGINLIB_EXPORT_CLASS(rr::GlobalCenterPathLayer, costmap_2d::Layer)
