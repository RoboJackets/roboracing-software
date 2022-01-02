#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rr_msgs/msg/axes.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std/vector.hpp>

namespace rr_gazebo {

float w, x, y, z;
float roll, pitch, yaw;
rr_msgs::msg::Axes::SharedPtr axes_msg;

class ImuAxisNode : public rclcpp::Node {
  public:
    explicit ImuAxisNode(const rclcpp::NodeOptions& options) : rclcpp::Node("imu_axes", options) {
        axes_pub_ = create_publisher<rr_msgs::msg::Axes>("/axes", rclcpp::SystemDefaultsQoS());

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
              "/imu", rclcpp::SystemDefaultsQoS(), std::bind(ImuAxisNode::imuCB, this, std::placeholders::_1));
    }

  private:
    rclcpp::Publisher<rr_msgs::msg::Axes>::SharedPtr axes_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    void toEulerianAngle(float w, float x, float y, float z) {
        double ysqr = y * y;

        double t0 = 2.0 * (w * x + y * z);
        double t1 = 1.0 - 2.0 * (x * x + ysqr);
        roll = atan2(t0, t1);

        double t2 = 2.0 * (w * y - z * x);
        t2 = t2 > 1.0 ? 1.0 : t2;
        t2 = t2 < -1.0 ? -1.0 : t2;
        pitch = asin(t2);

        double t3 = 2.0 * (w * z + x * y);
        double t4 = 1.0 - 2.0 * (ysqr + z * z);
        yaw = atan2(t3, t4);
        std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subs;

        rr_msgs::msg::Axes::SharedPtr publishable_copy = axes_msg;
        // publishable_copy->header.stamp = ros::Time::now();
        axes_pub_->publish(*publishable_copy);
    }

    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg) {
        w = msg->orientation.w;
        x = msg->orientation.x;
        y = msg->orientation.y;
        z = msg->orientation.z;

        toEulerianAngle(w, x, y, z);

        axes_msg->roll = roll;
        axes_msg->pitch = pitch;
        axes_msg->yaw = yaw;
    }
};  // ImuAxisNode

}  // namespace rr_gazebo

RCLCPP_COMPONENTS_REGISTER_NODE(rr_gazebo::ImuAxisNode);

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rr_gazebo::ImuAxisNode>();
    rclcpp::Rate rate(30);
    // rclcpp::spin(node);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        rate.sleep();
    }
    rclcpp::shutdown();
}

// if this is being buggy then try setting the rate to 30hz