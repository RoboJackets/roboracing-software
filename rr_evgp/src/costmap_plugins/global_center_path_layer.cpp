#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <dynamic_reconfigure/server.h>
#include <parameter_assertions/assertions.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <rr_evgp/UniformCostSearch.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Path.h>


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
        ros::NodeHandle private_nh("~" + getName());
        std::string pathOutputTopic;
        assertions::param(private_nh, "output_topic", pathOutputTopic, std::string("/global_center_path"));
        pub_path = nh.advertise<nav_msgs::Path>(pathOutputTopic, 1);

        assertions::getParam(private_nh, "wall_length", wall_length_, { assertions::greater<double>(0) });
        assertions::getParam(private_nh, "wall_distance_behind_start", wall_distance_behind_start_, { assertions::greater<double>(0) });
        assertions::getParam(private_nh, "wall_thickness", wall_thickness_, { assertions::greater<double>(0) });
        assertions::getParam(private_nh, "scaling_factor", scaling_factor_, { assertions::greater_eq<double>(1) });


        assertions::getParam(private_nh, "distance_from_start", distance_from_start_, { assertions::greater<double>(0) });

    }

    void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                      double* max_y) override {
        if (!enabled_) {
            return;
        }


        auto* costmap = layered_costmap_->getCostmap();
        double resolution = costmap->getResolution();


        curr_robot_x = robot_x;
        curr_robot_y = robot_y;
        if (!init_is_set) {
            init_robot_x = robot_x + 10; //#TODO launch params for extra offset and convert to real units
            init_robot_y = robot_y;
            init_robot_yaw = robot_yaw;
            init_is_set = true;
        }

    }

    void updateCosts(costmap_2d::Costmap2D& master_grid, int min_cell_x, int min_cell_y, int max_cell_x,
                     int max_cell_y) override {
        if (!enabled_) {
            return;
        }
/*
        const double resolution = master_grid.getResolution();
        const double world_min_x = master_grid.getOriginX();
        const double world_max_x = world_min_x + master_grid.getSizeInMetersX();
        const double world_min_y = master_grid.getOriginY();
        const double world_max_y = world_min_y + master_grid.getSizeInMetersY();

        for (int mx = min_cell_x; mx < max_cell_x; mx++) {
            for (int my = min_cell_y; my < max_cell_y; my++) {
                updates_[master_grid.getIndex(mx, my)] = -1;
            }
        }
*/
        double dx = curr_robot_x - init_robot_x;
        double dy = curr_robot_y - init_robot_y;
        double current_distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)); //c^2 = a^2 + b^2

        if (current_distance > distance_from_start_ + 1.0) { //#TODO:should this extra buffer be a param?
            has_left_initial_region = true;
        }

        if (has_left_initial_region) {
            ROS_WARN_STREAM("$$%%%%%%%Dist: " << current_distance);

            if (current_distance < distance_from_start_) {
                lap_completed = true;
            }
        }

        if (lap_completed) {

            if (!center_path_calculated) {
                center_path_calculated = true;
                ROS_WARN_STREAM("$$ PROCESSING IMAGE");

                //convert to opencv matrix
                cv::Mat mat_grid(master_grid.getSizeInCellsX(), master_grid.getSizeInCellsY(), CV_8UC1);
                uint8_t* charMap = master_grid.getCharMap();
                for (int r = 0; r < mat_grid.rows; r++) {
                    for (int c = 0; c < mat_grid.cols; c++) {
                        mat_grid.at<uint8_t>(r, c) = (charMap[r * mat_grid.cols + c] != costmap_2d::FREE_SPACE) ? 255 : 0;
                    }
                }

                //draw a wall to block the backwards path
                unsigned int wallLengthPixels = master_grid.cellDistance(wall_length_);
                unsigned int wallDistanceBehindStartPixels = master_grid.cellDistance(wall_distance_behind_start_);
                unsigned int wallThicknessPixels = master_grid.cellDistance(wall_thickness_);
                int mx;
                int my;
                master_grid.worldToMapEnforceBounds(init_robot_x, init_robot_y, mx, my);
                cv::Point2d wallMidpoint;
                wallMidpoint.x = -1.0 * std::cos(init_robot_yaw) * wallDistanceBehindStartPixels + mx;
                wallMidpoint.y = -1.0 * std::sin(init_robot_yaw) * wallDistanceBehindStartPixels + my;
                cv::Point2d wallPtA;
                cv::Point2d wallPtB;
                wallPtA.x = std::cos(init_robot_yaw + CV_PI/2.0) * wallLengthPixels/2.0 + wallMidpoint.x;
                wallPtA.y = std::sin(init_robot_yaw + CV_PI/2.0) * wallLengthPixels/2.0 + wallMidpoint.y;
                wallPtB.x = std::cos(init_robot_yaw - CV_PI/2.0) * wallLengthPixels/2.0 + wallMidpoint.x;
                wallPtB.y = std::sin(init_robot_yaw - CV_PI/2.0) * wallLengthPixels/2.0 + wallMidpoint.y;


                double goalDistanceBehindStart = 30; //#TODO: launch param
                cv::Point2d goalPt;
                goalPt.x = -1.0 * std::cos(init_robot_yaw) * goalDistanceBehindStart + mx; //#TODO: way to search for nearest open pt in case that pt is blocked
                goalPt.y = -1.0 * std::sin(init_robot_yaw) * goalDistanceBehindStart + my;

                cv::Mat debug;
                cv::cvtColor(mat_grid, debug, CV_GRAY2BGR);
                int cx;
                int cy;
                master_grid.worldToMapEnforceBounds(curr_robot_x, curr_robot_y, cx, cy);
                
                cv::line(debug, wallPtA, wallPtB, CV_RGB(255,0,0), wallThicknessPixels); //add wall in between start and end points

                cv::imwrite("/home/brian/catkin_ws/src/roboracing-software/rr_evgp/src/center_path_finder/CVTEST8.png", debug);

                cv::Mat channels[3];
                cv::split(debug, channels);

                cv::Mat obstacleGrid;
                cv::resize(channels[2], obstacleGrid, cv::Size(master_grid.getSizeInCellsX() / scaling_factor_, master_grid.getSizeInCellsY() / scaling_factor_));
                cv::threshold(obstacleGrid, obstacleGrid, 127, 255, cv::THRESH_BINARY); //threshold after scaling because of interpolation (halfway at 127 seems fine)

                //Ensures track walls are at least 2 pixels wide (can't sneak through) and adds buffer in case of badly sensed wall
                cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
                cv::dilate(obstacleGrid, obstacleGrid, kernel);


                cv::bitwise_not(obstacleGrid, obstacleGrid); //#TODO
                cv::Mat distanceGrid;
                cv::distanceTransform(obstacleGrid, distanceGrid, cv::DIST_L2, cv::DIST_MASK_3);
                //convert based on max so that we minimize the center path
                double min;
                double max;
                cv::minMaxLoc(distanceGrid, &min, &max);
                cv::absdiff(distanceGrid, cv::Scalar(max), distanceGrid);
                cv::bitwise_not(obstacleGrid, obstacleGrid); //#TODO

                cv::Point startPt(mx / scaling_factor_, my / scaling_factor_);
                goalPt.x = goalPt.x / scaling_factor_;
                goalPt.y = goalPt.y / scaling_factor_;
                UniformCostSearch ucs(obstacleGrid, distanceGrid, startPt, goalPt);

                //cv::Point newGoalPt = ucs.getNearestFreePointBFS(goalPt); //#TODO: try this
                //ucs.setGoalPoint(newGoalPt);

                std::vector<cv::Point> rawPointPath = ucs.search();

                //Display the Image for debug #TODO: remove
                cv::Mat displayImage;
                cv::cvtColor(obstacleGrid, displayImage, cv::COLOR_GRAY2BGR);
                for (cv::Point pt : rawPointPath) {
                    cv::Vec3b color(0,0,255);
                    displayImage.at<cv::Vec3b>(pt) = color;
                }
                displayImage.at<cv::Vec3b>(goalPt) = cv::Vec3b(255,0,0);
                //displayImage.at<cv::Vec3b>(newGoalPt) = cv::Vec3b(0,255,255);

                cv::imwrite("/home/brian/catkin_ws/src/roboracing-software/rr_evgp/src/center_path_finder/fullPath02.png", displayImage);

                pathMsg.header.stamp = ros::Time::now();
                pathMsg.header.frame_id = "map"; //#TODO: param
                for (cv::Point p : rawPointPath) { //#TODO stamps and orientation?
                    geometry_msgs::PoseStamped poseStamped;
                    //poseStamped.header.stamp
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
    double scaling_factor_;
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
