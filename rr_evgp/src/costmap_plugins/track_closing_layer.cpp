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
    cv::Mat img(rows, cols, CV_8UC1);
    for (size_t i = 0; i < tmp_a.size(); i++) {
        img.at<uint8_t>(i) = 255 * (int)tmp_a[i];
    }

    // remove staircases
    // shamelessly borrowed from https://github.com/yati-sagade/zhang-suen-thinning/blob/master/zhangsuen.cpp
    for (int iter = 0; iter < 2; iter++) {
        for (int i = 1; i < img.rows - 1; i++) {
            for (int j = 1; j < img.cols - 1; j++) {
                int c = img.at<uint8_t>(i, j);
                if (!c) {
                    continue;
                }
                int e = img.at<uint8_t>(i, j + 1), ne = img.at<uint8_t>(i - 1, j + 1), n = img.at<uint8_t>(i - 1, j),
                    nw = img.at<uint8_t>(i - 1, j - 1), w = img.at<uint8_t>(i, j - 1),
                    sw = img.at<uint8_t>(i + 1, j - 1), s = img.at<uint8_t>(i + 1, j),
                    se = img.at<uint8_t>(i + 1, j + 1);

                if (iter == 0) {
                    // North biased staircase removal
                    if ((n && ((e && !ne && !sw && (!w || !s)) || (w && !nw && !se && (!e || !s))))) {
                        img.at<uint8_t>(i, j) = 0;
                    }
                } else {
                    // South bias staircase removal
                    if ((s && ((e && !se && !nw && (!w || !n)) || (w && !sw && !ne && (!e || !n))))) {
                        img.at<uint8_t>(i, j) = 0;
                    }
                }
            }
        }
    }

    return img;
}

// take a skeletonized image and remove the small branches
cv::Mat removeSmallBranches(const cv::Mat& img) {
    constexpr int min_branch_length = 5;

    cv::Mat line_kernel(3, 3, CV_8U, cv::Scalar(1));
    line_kernel.at<uint8_t>(1, 1) = 10;

    cv::Mat filtered_img;
    cv::filter2D(img / 255, filtered_img, CV_16S, line_kernel);

    cv::Mat branches;
    cv::inRange(filtered_img, cv::Scalar(11), cv::Scalar(12), branches);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(branches, contours, CV_RETR_LIST, CV_CHAIN_APPROX_TC89_KCOS);

    cv::Mat big_branches(img.rows, img.cols, CV_8U, cv::Scalar(0));
    for (size_t i = 0; i < contours.size(); i++) {
        if (cv::arcLength(contours[i], false) >= min_branch_length) {
            cv::drawContours(big_branches, contours, i, cv::Scalar(255), 1);
        }
    }

    for (size_t i = 0; i < img.rows * img.cols; i++) {
        if (filtered_img.at<int16_t>(i) >= 13) {
            big_branches.at<uint8_t>(i) = 255;
        }
    }

    return big_branches;
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

    auto skel = thinObstacles(master_grid);
    auto branches = removeSmallBranches(skel);

    cv::imshow("skel", skel);
    cv::imshow("branch", branches);
    cv::waitKey(10);
}

}  // namespace rr
