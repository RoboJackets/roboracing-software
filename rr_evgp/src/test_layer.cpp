#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>
#include <parameter_assertions/assertions.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace rr {

class TestLayer : public costmap_2d::Layer {
  public:
    TestLayer() = default;

    void onInitialize() override {
        dsrv_ = std::make_unique<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>>();
        dsrv_->setCallback(boost::bind(&TestLayer::reconfigureCB, this, _1, _2));

        ros::NodeHandle private_nh("~" + name_);

        assertions::Assertion<double> assert_is_prob([](double x) { return x > 0 && x < 1; }, "probability in (0, 1) "
                                                                                              "range");
        assertions::getParam(private_nh, "prob_false_pos", prob_false_pos_, { assert_is_prob });
        assertions::getParam(private_nh, "prob_false_neg", prob_false_neg_, { assert_is_prob });
        assertions::getParam(private_nh, "prob_grid_prior", prob_grid_prior_, { assert_is_prob });
    }

    void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                      double* max_y) override {
        robot_x_ = robot_x;
        robot_y_ = robot_y;
    }

    void updateCosts(costmap_2d::Costmap2D& master_grid, int min_cell_x, int min_cell_y, int max_cell_x,
                     int max_cell_y) override {
        if (!enabled_)
            return;

        // TODO don't assume master map is stationary
        if (probs_.size() != master_grid.getSizeInCellsX() * master_grid.getSizeInCellsY()) {
            probs_.resize(master_grid.getSizeInCellsX() * master_grid.getSizeInCellsY());
            for (double& p : probs_)
                p = prob_grid_prior_;
        }

        min_cell_x = std::max(0, min_cell_x - 1);
        max_cell_x = std::min(max_cell_x + 1, (int)master_grid.getSizeInCellsX() - 1);
        min_cell_y = std::max(0, min_cell_y - 1);
        max_cell_y = std::min(max_cell_y + 1, (int)master_grid.getSizeInCellsY() - 1);
        for (int mx = min_cell_x; mx <= max_cell_x; mx++) {
            for (int my = min_cell_y; my <= max_cell_y; my++) {
                unsigned int i = master_grid.getIndex(mx, my);
                constexpr double eps = 1e-20;

                auto obstacle_layer_cost = master_grid.getCost(mx, my);
                if (obstacle_layer_cost == costmap_2d::FREE_SPACE) {
                    double p_prior = probs_[i];
                    double p_occupied = prob_false_neg_ * (p_prior + eps);
                    double p_empty = (1.0 - prob_false_pos_) * (1.0 - p_prior + eps);
                    probs_[i] = p_occupied / (p_occupied + p_empty);
                } else if (obstacle_layer_cost > 100 && obstacle_layer_cost != costmap_2d::NO_INFORMATION) {
                    double p_prior = probs_[i];
                    double p_occupied = (1.0 - prob_false_neg_) * (p_prior + eps);
                    double p_empty = prob_false_pos_ * (1.0 - p_prior + eps);
                    probs_[i] = p_occupied / (p_occupied + p_empty);
                }

                //                double wx, wy;
                //                master_grid.mapToWorld(mx, my, wx, wy);
                //                if (std::pow(wx - robot_x_, 2) + std::pow(wy - robot_y_, 2) < 5.0)

                if (obstacle_layer_cost != costmap_2d::NO_INFORMATION) {
                    master_grid.setCost(mx, my, 1 + static_cast<uint8_t>(probs_[i] * 99));
                }
            }
        }
    }

  private:
    void reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level) {
        enabled_ = config.enabled;
    }

    std::vector<double> probs_;
    double prob_false_pos_;
    double prob_false_neg_;
    double prob_grid_prior_;

    double robot_x_;
    double robot_y_;

    std::unique_ptr<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>> dsrv_;
};

}  // namespace rr

PLUGINLIB_EXPORT_CLASS(rr::TestLayer, costmap_2d::Layer)
