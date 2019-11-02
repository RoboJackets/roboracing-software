#include <valarray>

#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <dynamic_reconfigure/server.h>
#include <parameter_assertions/assertions.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include "skeletonize.hpp"


namespace rr {

class TrackClosingLayer : public costmap_2d::Layer {
  public:
    TrackClosingLayer() = default;

    void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                      double* max_y) override;

    void updateCosts(costmap_2d::Costmap2D& master_grid, int min_cell_x, int min_cell_y, int max_cell_x,
                     int max_cell_y) override;

  protected:
    void onInitialize() override;

  private:
    std::unique_ptr<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>> dsrv_;
    int branch_pruning_size_;
    int dilate_size_;

    void reconfigureCB(const costmap_2d::GenericPluginConfig& config, uint32_t level) {
        enabled_ = config.enabled;
    }
};

PLUGINLIB_EXPORT_CLASS(rr::TrackClosingLayer, costmap_2d::Layer)

// ******************* Implementation ********************** //


void TrackClosingLayer::onInitialize() {
    dsrv_ = std::make_unique<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>>();
    dsrv_->setCallback(boost::bind(&TrackClosingLayer::reconfigureCB, this, _1, _2));

    ros::NodeHandle private_nh("~" + getName());
     assertions::getParam(private_nh, "branch_pruning_size", branch_pruning_size_, { assertions::greater<int>(0) });
     assertions::getParam(private_nh, "dilate_size", dilate_size_, { assertions::greater<int>(0) });
}

void TrackClosingLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                     double* max_x, double* max_y) {
    if (!enabled_) {
        return;
    }

    const costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    *min_x = std::min(*min_x, costmap->getOriginX());
    *min_y = std::min(*min_y, costmap->getOriginY());
    *max_x = std::max(*max_x, costmap->getOriginX() + costmap->getSizeInMetersX());
    *max_y = std::max(*max_y, costmap->getOriginY() + costmap->getSizeInMetersY());
}

void TrackClosingLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_cell_x, int min_cell_y, int max_cell_x,
                                    int max_cell_y) {
    if (!enabled_) {
        return;
    }

    cv::Mat mat_grid(master_grid.getSizeInCellsX(), master_grid.getSizeInCellsY(), CV_8UC1);
    uint8_t* charMap = master_grid.getCharMap();
    for (int r = 0; r < mat_grid.rows; r++) {
        for (int c = 0; c < mat_grid.cols; c++) {
            mat_grid.at<uint8_t>(r, c) = (charMap[r * mat_grid.cols + c] == costmap_2d::LETHAL_OBSTACLE) ? 255 : 0;
        }
    }

    cv::Mat display_img(mat_grid.rows, mat_grid.cols * 4, CV_8UC1, cv::Scalar(255));
    cv::Rect roi(0, 0, mat_grid.cols, mat_grid.rows);

    mat_grid.copyTo(display_img(roi));

    auto dilate_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilate_size_, dilate_size_));
    cv::morphologyEx(mat_grid, mat_grid, cv::MORPH_DILATE, dilate_kernel);

    roi.x += mat_grid.cols;
    mat_grid.copyTo(display_img(roi));

    auto skel = rr::thinObstacles(mat_grid);
    auto branches = rr::removeSmallBranches(skel, branch_pruning_size_);

    roi.x += mat_grid.cols;
    skel.copyTo(display_img(roi));
    roi.x += mat_grid.cols;
    branches.copyTo(display_img(roi));
    cv::imshow("closing", display_img);
    cv::waitKey(1);
}

}  // namespace rr
