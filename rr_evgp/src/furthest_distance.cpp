#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

using std::placeholders::_1;

class FurthestPoint : public rclcpp::Node
{ 
public:
  FurthestPoint(const std::string & name)
  : Node(name) 
  {
    pos_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&FurthestPoint::pos_callback, this, _1)
    );
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "global_costmap/costmap", 10, std::bind(&FurthestPoint::map_callback, this, _1));
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    
  }

private:
  double euclid_distance(double x1, double y1, double x2, double y2){
     int x_dist = x2 - x1;
     int y_dist = y2 - y1;
     return sqrt((double)((x_dist*x_dist)+(y_dist*y_dist)));
  }

  void pos_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pose_ = msg->pose.pose;
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msgs)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msgs->header;
    double distance = 0;
    double x_meters = 0;
    double y_meters = 0;
    unsigned int robot_map_x = 0;
    unsigned int robot_map_y = 0;
    nav2_costmap_2d::Costmap2D costmap = nav2_costmap_2d::Costmap2D(*msgs);
    costmap.worldToMap(robot_pose_.position.x, robot_pose_.position.y, robot_map_x, robot_map_y);
  
    for (unsigned int x = 0; x < costmap.getSizeInCellsX(); x++) {
      for (unsigned int y = 0; y < costmap.getSizeInCellsY(); y++) {
        unsigned char cost = costmap.getCost(x, y);
        double curr_distance = euclid_distance(x, y, robot_map_x, robot_map_y);
        if (cost < 140 &&  curr_distance > distance ) {
          double temp_x, temp_y;
          costmap.mapToWorld(x, y, temp_x, temp_y);
          if (euclid_distance(goal_pose_.pose.position.x, goal_pose_.pose.position.y, temp_x, temp_y) < 1) {
            distance = curr_distance;
            x_meters = temp_x;
            y_meters = temp_y;
          }
        }
      }
    }
    if (distance > 0) {
      pose.pose.position.x = x_meters;
      pose.pose.position.y = y_meters;
      pose.pose.position.z = 0;
      pose.pose.orientation.x = 0;
      pose.pose.orientation.y = 0;
      pose.pose.orientation.z = 0;
      pose.pose.orientation.w = 1;
      
      goal_pose_ = pose;
      RCLCPP_INFO(this->get_logger(), "publisher publishing");
      pub_->publish(pose);
    }
  }
  
  

private:  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pos_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  geometry_msgs::msg::Pose robot_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto furthest_point_to_pose = std::make_shared<FurthestPoint>("furthest_point");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(furthest_point_to_pose);
 
  executor.spin();

  rclcpp::shutdown();

  return 0;
}