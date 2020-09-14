#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <dynamic_reconfigure/server.h>
#include <parameter_assertions/assertions.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <valarray>

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
    double min_enclosing_radius_;
    int reg_bounding_box_size_;
    double extrapolate_distance_;
    double max_angle_between_lines_;
    double max_dist_between_lines_;
    int wall_output_dilation_;

    void reconfigureCB(const costmap_2d::GenericPluginConfig& config, uint32_t level) {
        enabled_ = config.enabled;
    }

    using WallEndpointContext = std::tuple<cv::Point, cv::Point, double, double>;
    bool IsConnectable(const WallEndpointContext& current, const WallEndpointContext& prev);
};

PLUGINLIB_EXPORT_CLASS(rr::TrackClosingLayer, costmap_2d::Layer)

// ******************* Implementation ********************** //

void TrackClosingLayer::onInitialize() {
    dsrv_ = std::make_unique<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>>();
    dsrv_->setCallback(boost::bind(&TrackClosingLayer::reconfigureCB, this, _1, _2));

    ros::NodeHandle private_nh("~" + getName());
    assertions::getParam(private_nh, "branch_pruning_size", branch_pruning_size_, { assertions::greater<int>(0) });
    assertions::getParam(private_nh, "dilate_size", dilate_size_, { assertions::greater_eq<int>(0) });
    assertions::getParam(private_nh, "min_enclosing_radius", min_enclosing_radius_, { assertions::greater<double>(0) });
    assertions::getParam(private_nh, "reg_bounding_box_size", reg_bounding_box_size_, { assertions::greater<int>(0) });
    assertions::getParam(private_nh, "extrapolate_distance", extrapolate_distance_, { assertions::greater<double>(0) });
    assertions::getParam(private_nh, "max_angle_between_lines", max_angle_between_lines_,
                         { assertions::greater<double>(0) });
    assertions::getParam(private_nh, "max_dist_between_lines", max_dist_between_lines_,
                         { assertions::greater<double>(0) });
    assertions::getParam(private_nh, "wall_output_dilation", wall_output_dilation_, { assertions::greater_eq<int>(0) });
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

template <int N>
double norm(const cv::Vec<double, N>& x) {
    return std::sqrt(x.dot(x));
}

cv::Point2d intersect_point_line(const cv::Point2d& p, const cv::Point2d& l1, const cv::Point2d& l2, bool bound) {
    cv::Vec2d n = l2 - l1;
    n /= norm(n);
    double t = (p - l1).dot(n);
    cv::Vec2d x = cv::Vec2d(l1) + n * t;

    if (bound && (norm(x - cv::Vec2d(l1)) + norm(x - cv::Vec2d(l2)) > norm(l2 - l1))) {
        x = (norm(p - l1) < norm(p - l2)) ? l1 : l2;
    }

    return x;
}

bool TrackClosingLayer::IsConnectable(const WallEndpointContext& current, const WallEndpointContext& prev) {
    const auto& [endpntA, pntA, vxA, vyA] = current;
    const auto& [endpntB, pntB, vxB, vyB] = prev;

    double angle_between_lines = 180.0 - (std::acos(vxA * vxB + vyA * vyB) * 180.0 / M_PI);

    cv::Point closest_endB_to_lineA = intersect_point_line(endpntB, pntA, pntA + cv::Point(vxA * 500, vyA * 500), true);
    cv::Point closest_endA_to_lineB = intersect_point_line(endpntA, pntB, pntB + cv::Point(vxB * 500, vyB * 500), true);
    double dist_a = norm(closest_endA_to_lineB - endpntA);
    double dist_b = norm(closest_endB_to_lineA - endpntB);

    return angle_between_lines <= max_angle_between_lines_ && dist_a < max_dist_between_lines_ &&
           dist_b < max_dist_between_lines_;
}

void TrackClosingLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_cell_x, int min_cell_y, int max_cell_x,
                                    int max_cell_y) {
    if (!enabled_) {
        return;
    }

    // populate OpenCV Mat as grid
    cv::Mat mat_grid(master_grid.getSizeInCellsX(), master_grid.getSizeInCellsY(), CV_8UC1);
    uint8_t* charMap = master_grid.getCharMap();
    for (int r = 0; r < mat_grid.rows; r++) {
        for (int c = 0; c < mat_grid.cols; c++) {
            mat_grid.at<uint8_t>(r, c) = (charMap[r * mat_grid.cols + c] == costmap_2d::LETHAL_OBSTACLE) ? 255 : 0;
        }
    }

    if (dilate_size_ > 0) {
        int kernel_size = 2 * dilate_size_ + 1;
        auto dilate_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
        cv::morphologyEx(mat_grid, mat_grid, cv::MORPH_DILATE, dilate_kernel);

        if (dilate_size_ > 1) {
            kernel_size = 2 * dilate_size_ - 1;
            auto erode_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
            cv::morphologyEx(mat_grid, mat_grid, cv::MORPH_ERODE, erode_kernel);
        }
    }

    // Make Skeleton
    auto skel = rr::thinObstacles(mat_grid);
    auto branches = rr::removeSmallBranches(skel, branch_pruning_size_);

    // cv::Mat debug_img;
    // cv::cvtColor(branches * 255, debug_img, CV_GRAY2BGR);

    // Remove small objects (cars and noise)
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(branches, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    cv::Mat contour_img(skel.rows, skel.cols, CV_8UC1, cv::Scalar(0));
    for (size_t i = 0; i < contours.size(); i++) {
        if (cv::arcLength(contours[i], false) >= min_enclosing_radius_) {
            cv::drawContours(contour_img, contours, i, cv::Scalar(1), 1);
        }
    }

    // Get end points
    cv::Mat line_kernel(3, 3, CV_8UC1, cv::Scalar(1));
    line_kernel.at<uint8_t>(1, 1) = 10;
    cv::Mat filtered;
    cv::filter2D(contour_img, filtered, CV_16SC1, line_kernel);

    std::vector<cv::Point> endpoints;
    cv::inRange(filtered, cv::Scalar(11), cv::Scalar(11), filtered);
    cv::findNonZero(filtered, endpoints);

    std::vector<WallEndpointContext> endpnt_line;
    const size_t half_size = reg_bounding_box_size_ / 2;
    const cv::Point half_size_point(half_size, half_size);

    for (const cv::Point& endpnt : endpoints) {
        cv::Point tl = endpnt - half_size_point;
        cv::Point br = endpnt + half_size_point;
        tl.x = std::max(tl.x, 0);
        tl.y = std::max(tl.y, 0);
        br.x = std::min(br.x, contour_img.cols - 1);
        br.y = std::min(br.y, contour_img.rows - 1);

        // cv::rectangle(debug_img, tl, br, cv::Scalar(255, 0, 0), 1);

        // Find surrounding points
        cv::Mat endpnt_box = contour_img(cv::Rect(tl, br));
        std::vector<cv::Point> box_points;
        cv::findNonZero(endpnt_box, box_points);

        // Get regression line
        cv::Vec4f line;
        cv::fitLine(box_points, line, cv::DIST_L2, 0, 0.01, 0.01);
        double vx = line(0);
        double vy = line(1);
        double x = line(2) + tl.x;
        double y = line(3) + tl.y;

        // Get direction of line from end point
        cv::Point2d new_endpnt = intersect_point_line(endpnt, cv::Point2d(x, y), cv::Point2d(x + vx, y + vy), false);

        cv::Vec2d avg_pnt;
        for (const cv::Point& pnt : box_points) {
            avg_pnt(0) += pnt.x + tl.x;
            avg_pnt(1) += pnt.y + tl.y;
        }
        avg_pnt /= static_cast<double>(box_points.size());

        cv::Vec2d v(vx, vy);
        double projection = (cv::Vec2d(new_endpnt) - avg_pnt).dot(v);
        if (projection < 0) {
            vx *= -1;
            vy *= -1;
        }

        // cv::circle(debug_img, new_endpnt, 2, cv::Scalar(0, 255, 0));
        cv::Point2d point;
        point.x = new_endpnt.x + extrapolate_distance_ * vx;
        point.y = new_endpnt.y + extrapolate_distance_ * vy;
        endpnt_line.emplace_back(new_endpnt, point, vx, vy);
    }

    // Connect end points
    cv::Mat walls(mat_grid.rows, mat_grid.cols, CV_8UC1, cv::Scalar(0));
    std::valarray<bool> have_connection(false, endpnt_line.size());
    for (size_t i = 0; i < endpnt_line.size(); i++) {
        const auto& curr_line_context = endpnt_line.at(i);
        for (size_t j = i + 1; j < endpnt_line.size(); j++) {
            const auto& comp_line_context = endpnt_line.at(j);
            if (IsConnectable(curr_line_context, comp_line_context)) {
                have_connection[i] = true;
                have_connection[j] = true;
                const auto& point_a = std::get<0>(curr_line_context);
                const auto& point_b = std::get<0>(comp_line_context);
                // cv::line(debug_img, point_a, point_b, cv::Scalar(255, 0, 255));
                cv::line(walls, point_a, point_b, cv::Scalar(255));
            }
        }

        if (!have_connection[i]) {
            const auto& endpnt = std::get<0>(curr_line_context);
            const auto& extrapolated_pnt = std::get<1>(curr_line_context);
            // cv::line(debug_img, endpnt, extrapolated_pnt, cv::Scalar(0, 0, 255));
            cv::line(walls, endpnt, extrapolated_pnt, cv::Scalar(255));
        }
    }

    // make walls larger if desired
    if (wall_output_dilation_ > 0) {
        int ksize = 2 * wall_output_dilation_ + 1;
        auto wall_dilate_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ksize, ksize));
        cv::morphologyEx(walls, walls, cv::MORPH_DILATE, wall_dilate_kernel);
    }

    // Make map from image
    for (int r = 0; r < walls.rows; r++) {
        for (int c = 0; c < walls.cols; c++) {
            if (walls.at<uint8_t>(r, c) > 0) {
                charMap[r * walls.cols + c] = costmap_2d::LETHAL_OBSTACLE;
            }
        }
    }

    // cv::imshow("closing", debug_img);
    // cv::waitKey(1);
}

}  // namespace rr
