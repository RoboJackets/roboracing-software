#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>
#include <parameter_assertions/assertions.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace rr {

struct Pose2D {
    double x;
    double y;
    double theta;
};

class TrackClosingLayer : public costmap_2d::Layer {
  public:
    TrackClosingLayer() : extrapolate_dist_(0), interpolate_dist_(0), robot_pose() {}

    void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                      double* max_y) override;

    void updateCosts(costmap_2d::Costmap2D& master_grid, int min_cell_x, int min_cell_y, int max_cell_x,
                     int max_cell_y) override;

  protected:
    void onInitialize() override;

  private:
    double extrapolate_dist_;
    double interpolate_dist_;

    std::unique_ptr<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>> dsrv_;
    Pose2D robot_pose;

    void reconfigureCB(const costmap_2d::GenericPluginConfig& config, uint32_t level) {
        enabled_ = config.enabled;
    }
};

void TrackClosingLayer::onInitialize() {
    dsrv_ = std::make_unique<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>>();
    dsrv_->setCallback(boost::bind(&TrackClosingLayer::reconfigureCB, this, _1, _2));

    ros::NodeHandle private_nh("~" + getName());
    assertions::getParam(private_nh, "extrapolate_dist", extrapolate_dist_, { assertions::greater<double>(0) });
    assertions::getParam(private_nh, "interpolate_dist", interpolate_dist_, { assertions::greater<double>(0) });
}

void TrackClosingLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                     double* max_x, double* max_y) {
    if (!enabled_) {
        return;
    }

    double resolution = layered_costmap_->getCostmap()->getResolution();
    double max_update_dist = std::max(extrapolate_dist_, interpolate_dist_) + resolution;

    *min_x -= max_update_dist;
    *min_y -= max_update_dist;
    *max_x += max_update_dist;
    *max_y += max_update_dist;

    robot_pose.x = robot_x;
    robot_pose.y = robot_y;
    robot_pose.theta = robot_yaw;
}

void TrackClosingLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_cell_x, int min_cell_y, int max_cell_x,
                                    int max_cell_y) {}

}  // namespace rr

PLUGINLIB_EXPORT_CLASS(rr::TrackClosingLayer, costmap_2d::Layer)
