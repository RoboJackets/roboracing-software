#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <dynamic_reconfigure/server.h>
#include <parameter_assertions/assertions.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <rr_evgp/UniformCostSearch.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>


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


        //#REMOVE debug
        ros::NodeHandle nh;
        debug_pub = nh.advertise<sensor_msgs::Image>("/aa/centerpathdebug", 1);

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
        counter++;
        if (counter > 1100) {
            double dx = std::fabs(curr_robot_x - init_robot_x);
            double dy = std::fabs(curr_robot_y - init_robot_y);

            ROS_WARN_STREAM_THROTTLE(5, "###DX+DY: " << dx+dy);

            if (dx + dy < 30) {
                lap_completed = true;
            }
        }
        if (lap_completed) {
            //ROS_INFO_STREAM("$$$$$$ LAP COMPLETE $$$$$");

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
                double wallLength = 60; //#TODO: convert to real units
                double wallDistanceBehindStart = 10;
                double wallThickness = 3; //#TODO: convert to real
                int mx;
                int my;
                master_grid.worldToMapEnforceBounds(init_robot_x, init_robot_y, mx, my);
                cv::Point2d wallMidpoint;
                wallMidpoint.x = -1.0 * std::cos(init_robot_yaw) * wallDistanceBehindStart + mx;
                wallMidpoint.y = -1.0 * std::sin(init_robot_yaw) * wallDistanceBehindStart + my;
                cv::Point2d wallPtA;
                cv::Point2d wallPtB;
                wallPtA.x = std::cos(init_robot_yaw + CV_PI/2.0) * wallLength/2.0 + wallMidpoint.x;
                wallPtA.y = std::sin(init_robot_yaw + CV_PI/2.0) * wallLength/2.0 + wallMidpoint.y;
                wallPtB.x = std::cos(init_robot_yaw - CV_PI/2.0) * wallLength/2.0 + wallMidpoint.x;
                wallPtB.y = std::sin(init_robot_yaw - CV_PI/2.0) * wallLength/2.0 + wallMidpoint.y;


                double goalDistanceBehindStart = 30; //#TODO: launch param
                cv::Point2d goalPt;
                goalPt.x = -1.0 * std::cos(init_robot_yaw) * goalDistanceBehindStart + mx;
                goalPt.y = -1.0 * std::sin(init_robot_yaw) * goalDistanceBehindStart + my;

                cv::Mat debug;
                cv::cvtColor(mat_grid, debug, CV_GRAY2BGR);
                int cx;
                int cy;
                master_grid.worldToMapEnforceBounds(curr_robot_x, curr_robot_y, cx, cy);
                //cv::line(debug, cv::Point2d(cx, cy), wallMidpoint, CV_RGB(0,255,0),wallThickness);
                cv::line(debug, wallPtA, wallPtB, CV_RGB(255,0,0), wallThickness); //add wall in between start and end points

                cv::imwrite("/home/brian/catkin_ws/src/roboracing-software/rr_evgp/src/center_path_finder/CVTEST8.png", debug);

                cv::Mat channels[3];
                cv::split(debug, channels);
                //cv::Rect cropRect = findCropRect(channels[2]); //only red channel
                cv::Mat obstacleGrid;
                cv::resize(channels[2], obstacleGrid, cv::Size(500,500)); //#TODO: right now this is a scaling factor of 1/4
                cv::threshold(obstacleGrid, obstacleGrid, 127, 255, cv::THRESH_BINARY); //threshold after scaling because of interpolation (halfway at 127 seems fine)

                //Ensures track walls are at least 2 pixels wide (can't sneak through) and adds buffer in case of badly sensed wall
                cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
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

                cv::Point startPt(mx / 4, my / 4); //#TODO
                UniformCostSearch ucs(obstacleGrid, distanceGrid, startPt, goalPt);
                std::vector<cv::Point> rawPointPath = ucs.search();

                //Display the Image for debug #TODO: remove
                cv::Mat displayImage;
                cv::cvtColor(obstacleGrid, displayImage, cv::COLOR_GRAY2BGR);
                for (cv::Point pt : rawPointPath) {
                    cv::Vec3b color(0,0,255);
                    displayImage.at<cv::Vec3b>(pt) = color;
                }
                cv::imwrite("/home/brian/catkin_ws/src/roboracing-software/rr_evgp/src/center_path_finder/fullPath01.png", displayImage);


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
    int counter;
    bool lap_completed;
    bool init_is_set;
    bool center_path_calculated;
    ros::Publisher debug_pub;

    std::unique_ptr<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>> dsrv_;
};

}  // namespace rr

PLUGINLIB_EXPORT_CLASS(rr::GlobalCenterPathLayer, costmap_2d::Layer)
