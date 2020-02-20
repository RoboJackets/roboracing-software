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
            init_robot_x = robot_x;
            init_robot_y = robot_y;
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
        if (counter > 700) {
            double dx = std::fabs(curr_robot_x - init_robot_x);
            double dy = std::fabs(curr_robot_y - init_robot_y);

            //ROS_INFO_STREAM("@@@@@@@@@@ " << dx+dy);
            ROS_INFO_STREAM_THROTTLE(10, "###DX+DY: " << dx+dy);

            if (dx + dy < 30) {
                lap_completed = true;
            }
        }
        if (lap_completed) {
            //ROS_INFO_STREAM("$$$$$$ LAP COMPLETE $$$$$");

            if (!center_path_calculated) {
                center_path_calculated = true;

                cv::Mat mat_grid(master_grid.getSizeInCellsX(), master_grid.getSizeInCellsY(), CV_8UC1);
                uint8_t* charMap = master_grid.getCharMap();
                for (int r = 0; r < mat_grid.rows; r++) {
                    for (int c = 0; c < mat_grid.cols; c++) {
                        mat_grid.at<uint8_t>(r, c) = (charMap[r * mat_grid.cols + c] != costmap_2d::FREE_SPACE) ? 255 : 0;
                    }
                }
                ROS_INFO_STREAM("%%%%%%%%%%SIZE X CELLS: " << cv::countNonZero(mat_grid));
                cv::imwrite("/home/brian/catkin_ws/src/roboracing-software/rr_evgp/src/center_path_finder/CVTEST2.png",mat_grid);

                //master_grid.saveMap("/home/brian/catkin_ws/src/roboracing-software/rr_evgp/src/center_path_finder/test.pgm");
                ROS_INFO_STREAM("$$$$$$ DIDIIDIDIDIDIDI COMPLETE $$$$$");
                //cv::imshow("BIGTESTIMAGE", mat_grid);
                //cv::waitKey(1);
                //sensor_msgs::ImagePtr outmsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mat_grid).toImageMsg();
                //debug_pub.publish(outmsg);
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
    double curr_robot_x;
    double curr_robot_y;
    int counter;
    bool lap_completed;
    bool init_is_set;
    bool center_path_calculated;
    //UniformCostSearch UCS;
    ros::Publisher debug_pub;

    std::unique_ptr<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>> dsrv_;
};

}  // namespace rr

PLUGINLIB_EXPORT_CLASS(rr::GlobalCenterPathLayer, costmap_2d::Layer)
