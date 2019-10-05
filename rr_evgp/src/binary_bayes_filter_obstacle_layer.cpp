#include <valarray>

#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <dynamic_reconfigure/server.h>
#include <parameter_assertions/assertions.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

namespace rr {

class BinaryBayesFilterObstacleLayer : public costmap_2d::Layer {
  public:
    BinaryBayesFilterObstacleLayer() : tf_(), tf_listener_(tf_), ready_(false) {}

    void onInitialize() override {
        dsrv_ = std::make_unique<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>>();
        dsrv_->setCallback(boost::bind(&BinaryBayesFilterObstacleLayer::reconfigureCB, this, _1, _2));

        ros::NodeHandle global_nh;
        ros::NodeHandle costmap_nh("~costmap");
        ros::NodeHandle private_nh("~" + name_);

        assertions::Assertion<double> assert_is_prob([](double x) { return x > 0 && x < 1; }, "probability in (0, 1) "
                                                                                              "range");
        assertions::getParam(private_nh, "prob_false_pos", prob_false_pos_, { assert_is_prob });
        assertions::getParam(private_nh, "prob_false_neg", prob_false_neg_, { assert_is_prob });
        assertions::getParam(private_nh, "prob_grid_prior", prob_grid_prior_, { assert_is_prob });
        assertions::getParam(private_nh, "max_confidence", max_prob_, { assert_is_prob });
        assertions::getParam(private_nh, "scan_range", scan_range_, { assertions::greater<double>(0) });

        std::string scan_topic;
        assertions::getParam(private_nh, "scan_topic", scan_topic);
        scan_sub_ = global_nh.subscribe(scan_topic, 1, &BinaryBayesFilterObstacleLayer::scanCB, this);

        std::string lidar_frame, robot_base_frame;
        assertions::getParam(private_nh, "lidar_frame", lidar_frame);
        assertions::getParam(costmap_nh, "robot_base_frame", robot_base_frame);

        std::string err;
        while (!tf_.canTransform(robot_base_frame, lidar_frame, ros::Time(0), ros::Duration(1.0), &err)) {
            ROS_WARN_STREAM(err);
            ros::WallDuration(0.1).sleep();
        }

        auto transform_stamped = tf_.lookupTransform(robot_base_frame, lidar_frame, ros::Time(0), ros::Duration(1.0));
        lidar_offset_x_ = transform_stamped.transform.translation.x;
        lidar_offset_y_ = transform_stamped.transform.translation.y;
        lidar_offset_yaw_ = tf2::getYaw(transform_stamped.transform.rotation);

        ready_ = true;
    }

    void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                      double* max_y) override {
        if (!enabled_ || !ready_ || !most_recent_scan_) {
            return;
        }

        lidar_x_ = robot_x + (lidar_offset_x_ * std::cos(robot_yaw) - lidar_offset_y_ * std::sin(robot_yaw));
        lidar_y_ = robot_y + (lidar_offset_x_ * std::sin(robot_yaw) + lidar_offset_y_ * std::cos(robot_yaw));
        lidar_yaw_ = robot_yaw + lidar_offset_yaw_;

        scan_ = most_recent_scan_;

        double angle = scan_->angle_min;
        for (double range : scan_->ranges) {
            if (std::isinf(range)) {
                range = scan_range_;
            }
            double wx = lidar_x_ + range * std::cos(lidar_yaw_ + angle);
            double wy = lidar_y_ + range * std::sin(lidar_yaw_ + angle);
            angle += scan_->angle_increment;

            if (wx < *min_x)
                *min_x = wx;
            if (wx > *max_x)
                *max_x = wx;
            if (wy < *min_y)
                *min_y = wy;
            if (wy > *max_y)
                *max_y = wy;
        }

        double resolution = layered_costmap_->getCostmap()->getResolution();
        *min_x = std::max(*min_x - 3 * resolution, 0.0);
        *max_x = std::min(*max_x + 3 * resolution, layered_costmap_->getCostmap()->getSizeInMetersX() - 0.0001);
        *min_y = std::max(*min_y - 3 * resolution, 0.0);
        *max_y = std::min(*max_y + 3 * resolution, layered_costmap_->getCostmap()->getSizeInMetersY() - 0.0001);
    }

    void updateCosts(costmap_2d::Costmap2D& master_grid, int min_cell_x, int min_cell_y, int max_cell_x,
                     int max_cell_y) override {
        if (!enabled_ || !ready_ || !scan_) {
            return;
        }

        // TODO don't assume master map is stationary
        if (probs_.size() != master_grid.getSizeInCellsX() * master_grid.getSizeInCellsY()) {
            probs_.resize(master_grid.getSizeInCellsX() * master_grid.getSizeInCellsY());
            for (double& p : probs_) {
                p = prob_grid_prior_;
            }
            updates_.resize(probs_.size());
            ROS_INFO("resized probability map");
        }

        if (min_cell_x < 0)
            min_cell_x = 0;
        if (min_cell_y < 0)
            min_cell_y = 0;
        if (max_cell_x >= master_grid.getSizeInCellsX())
            max_cell_x = (int)master_grid.getSizeInCellsX() - 1;
        if (max_cell_y >= master_grid.getSizeInCellsY())
            max_cell_y = (int)master_grid.getSizeInCellsY() - 1;

        const double resolution = master_grid.getResolution();
        const double world_min_x = master_grid.getOriginX();
        const double world_max_x = world_min_x + master_grid.getSizeInMetersX();
        const double world_min_y = master_grid.getOriginY();
        const double world_max_y = world_min_y + master_grid.getSizeInMetersY();

        for (int mx = min_cell_x; mx <= max_cell_x; mx++) {
            for (int my = min_cell_y; my <= max_cell_y; my++) {
                updates_[master_grid.getIndex(mx, my)] = -1;
            }
        }

        double angle = scan_->angle_min;
        for (double raw_range : scan_->ranges) {
            bool is_hit = (!std::isinf(raw_range) && raw_range <= scan_range_);
            double range = is_hit ? raw_range : scan_range_;
            double trace_range = range + (resolution * 0.7);
            double wx = lidar_x_ + trace_range * std::cos(lidar_yaw_ + angle);
            double wy = lidar_y_ + trace_range * std::sin(lidar_yaw_ + angle);

            if (wx < world_min_x || wx >= world_max_x || wy < world_min_y || wy >= world_max_y) {
                ROS_WARN_THROTTLE(1.0, "trying to scan outside the map");
                continue;
            }

            const double step_size = resolution * 0.25;
            const double step_x = step_size * (wx - lidar_x_) / range;
            const double step_y = step_size * (wy - lidar_y_) / range;
            int mx, my;
            double d = 0;
            double march_x = lidar_x_;
            double march_y = lidar_y_;
            const double last_clear = range - resolution;
            while (d < last_clear) {
                master_grid.worldToMapNoBounds(march_x, march_y, mx, my);
                updates_[master_grid.getIndex(mx, my)] = 0;
                d += step_size;
                march_x += step_x;
                march_y += step_y;
            }
            if (is_hit) {
                while (d <= trace_range) {
                    master_grid.worldToMapNoBounds(march_x, march_y, mx, my);
                    updates_[master_grid.getIndex(mx, my)] = 1;
                    d += step_size;
                    march_x += step_x;
                    march_y += step_y;
                }
            }

            angle += scan_->angle_increment;
        }

        uint8_t* costmap_data = master_grid.getCharMap();
        for (int mx = min_cell_x; mx <= max_cell_x; mx++) {
            for (int my = min_cell_y; my <= max_cell_y; my++) {
                const auto i = master_grid.getIndex(mx, my);
                const char update = updates_[i];

                if (update >= 0) {
                    double p_prior = probs_[i];
                    if (update > 0) {  // hit
                        double p_occupied = (1.0 - prob_false_neg_) * p_prior;
                        double p_empty = prob_false_pos_ * (1.0 - p_prior);
                        probs_[i] = std::min(p_occupied / (p_occupied + p_empty), max_prob_);
                    } else {  // cleared
                        double p_occupied = prob_false_neg_ * p_prior;
                        double p_empty = (1.0 - prob_false_pos_) * (1.0 - p_prior);
                        probs_[i] = std::max(p_occupied / (p_occupied + p_empty), 1.0 - max_prob_);
                    }
                }

                costmap_data[i] = static_cast<uint8_t>(probs_[i] * 255.999);
            }
        }
    }

  private:
    void reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level) {
        enabled_ = config.enabled;
    }

    void scanCB(const sensor_msgs::LaserScanConstPtr& msg) {
        most_recent_scan_ = msg;
    }

    // state
    std::vector<double> probs_;
    std::vector<char> updates_;
    sensor_msgs::LaserScanConstPtr most_recent_scan_;
    sensor_msgs::LaserScanConstPtr scan_;
    double lidar_x_;
    double lidar_y_;
    double lidar_yaw_;
    double lidar_offset_x_;
    double lidar_offset_y_;
    double lidar_offset_yaw_;
    bool ready_;

    // params
    double prob_false_pos_;
    double prob_false_neg_;
    double prob_grid_prior_;
    double max_prob_;
    double scan_range_;

    std::unique_ptr<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>> dsrv_;
    ros::Subscriber scan_sub_;
    tf2_ros::Buffer tf_;
    tf2_ros::TransformListener tf_listener_;
};

}  // namespace rr

PLUGINLIB_EXPORT_CLASS(rr::BinaryBayesFilterObstacleLayer, costmap_2d::Layer)
