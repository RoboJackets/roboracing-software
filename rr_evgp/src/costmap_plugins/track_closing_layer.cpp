#include <valarray>

#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <dynamic_reconfigure/server.h>
#include <parameter_assertions/assertions.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

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

    void reconfigureCB(const costmap_2d::GenericPluginConfig& config, uint32_t level) {
        enabled_ = config.enabled;
    }
};

PLUGINLIB_EXPORT_CLASS(rr::TrackClosingLayer, costmap_2d::Layer)

// ******************* Implementation ********************** //

cv::Mat thinObstacles(const costmap_2d::Costmap2D& grid) {
    // Zhang thinning "A Fast Parallel Algorithm for Thinning Digital Patterns"
    // ref https://rosettacode.org/wiki/Zhang-Suen_thinning_algorithm#C.2B.2B

    int rows = grid.getSizeInCellsX();
    int cols = grid.getSizeInCellsY();
    const uint8_t* const grid_data = grid.getCharMap();

    std::valarray<bool> tmp_a(false, rows * cols);
    for (int r = 1; r < rows - 1; r++) {
        for (int c = 1; c < cols - 1; c++) {
            int i = r * cols + c;
            tmp_a[i] = (grid_data[i] == costmap_2d::LETHAL_OBSTACLE);
        }
    }

    std::valarray<bool> tmp_b = tmp_a;

    auto A = [cols](const std::valarray<bool>& data, int i) -> int {
        int sum = 0;
        sum += !data[i - cols] && data[i - cols + 1];  // 2 3
        sum += !data[i - cols + 1] && data[i + 1];     // 3 4
        sum += !data[i + 1] && data[i + cols + 1];     // 4 5
        sum += !data[i + cols + 1] && data[i + cols];  // 5 6
        sum += !data[i + cols] && data[i + cols - 1];  // 6 7
        sum += !data[i + cols - 1] && data[i - 1];     // 7 8
        sum += !data[i - 1] && data[i - cols - 1];     // 8 9
        sum += !data[i - cols - 1] && data[i - cols];  // 9 2
        return sum;
    };

    auto B = [cols](const std::valarray<bool>& data, int i) -> int {
        int sum = 0;
        sum += data[i - cols];
        sum += data[i - cols + 1];
        sum += data[i + 1];
        sum += data[i + cols + 1];
        sum += data[i + cols];
        sum += data[i + cols - 1];
        sum += data[i - 1];
        sum += data[i - cols - 1];
        return sum;
    };

    int changed_pixels;
    do {
        changed_pixels = 0;

        // step 1
        tmp_b = tmp_a;
        for (int r = 1; r < rows - 1; r++) {
            for (int c = 1; c < cols - 1; c++) {
                int i = r * cols + c;
                if (tmp_a[i]) {
                    int b = B(tmp_a, i);
                    if (b >= 2 && b <= 6 && A(tmp_a, i) == 1 && !(tmp_a[i - cols] && tmp_a[i + 1] && tmp_a[i + cols]) &&
                        !(tmp_a[i + 1] && tmp_a[i + cols] && tmp_a[i - 1])) {
                        tmp_b[i] = false;
                        changed_pixels++;
                    }
                }
            }
        }

        // step 2
        tmp_a = tmp_b;
        for (int r = 1; r < rows - 1; r++) {
            for (int c = 1; c < cols - 1; c++) {
                int i = r * cols + c;
                if (tmp_b[i]) {
                    int b = B(tmp_b, i);
                    if (b >= 2 && b <= 6 && A(tmp_b, i) == 1 && !(tmp_b[i - cols] && tmp_b[i + 1] && tmp_b[i - 1]) &&
                        !(tmp_b[i - cols] && tmp_b[i + cols] && tmp_b[i - 1])) {
                        tmp_a[i] = false;
                        changed_pixels++;
                    }
                }
            }
        }
    } while (changed_pixels > 0);

    // now tmp_a holds our solution
    cv::Mat out(rows, cols, CV_8UC1);
    for (size_t i = 0; i < tmp_a.size(); i++) {
        out.at<uint8_t>(i) = tmp_a[i];
    }
    return out;
}

void TrackClosingLayer::onInitialize() {
    dsrv_ = std::make_unique<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>>();
    dsrv_->setCallback(boost::bind(&TrackClosingLayer::reconfigureCB, this, _1, _2));

    ros::NodeHandle private_nh("~" + getName());
    // assertions::getParam(private_nh, "extrapolate_dist", extrapolate_dist_, { assertions::greater<double>(0) });
    // assertions::getParam(private_nh, "interpolate_dist", interpolate_dist_, { assertions::greater<double>(0) });
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

    cv::imshow("test", thinObstacles(master_grid) * 255);
    cv::waitKey(10);
}

}  // namespace rr
