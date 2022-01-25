#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rr_msgs/msg/speed.hpp>
#include <rr_msgs/msg/steering.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace rr_util {
class JoystickDriver : public rclcpp::Node {
  public:
    explicit JoystickDriver(const rclcpp::NodeOptions& options) : rclcpp::Node("joystick_driver", options) {
        JoystickDriver();
    }
    explicit JoystickDriver() : rclcpp::Node("joystick_driver") {
        // Subscribe to joystick_driver topic
        float_command_sub = create_subscription<sensor_msgs::msg::Joy>(
              "/joy", rclcpp::SystemDefaultsQoS(), std::bind(&JoystickDriver::JoystickCB, this, std::placeholders::_1));
        // Convert joystick_driver commands into motor commands on these topics
        speed_publisher = this->create_publisher<rr_msgs::msg::Speed>("/speed", 10);
        steering_publisher = this->create_publisher<rr_msgs::msg::Steering>("/steering", 10);

        // Driving limits
        angle_max = this->declare_parameter<double>("angle_max");
        speed_max = this->declare_parameter<double>("speed_max");
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr float_command_sub;
    rclcpp::Publisher<rr_msgs::msg::Speed>::SharedPtr speed_publisher;
    rclcpp::Publisher<rr_msgs::msg::Steering>::SharedPtr steering_publisher;
    double angle_max;
    double speed_max;
    void JoystickCB(const sensor_msgs::msg::Joy::SharedPtr msg) {
        rr_msgs::msg::Speed sp_cmd;
        rr_msgs::msg::Steering st_cmd;
        sp_cmd.speed = speed_max * ((-1 * msg->axes[5] + 1.) / 2.);
        st_cmd.angle = -(angle_max * msg->axes[0]);
        RCLCPP_INFO(this->get_logger(), "going to callback");
        speed_publisher->publish(sp_cmd);
        steering_publisher->publish(st_cmd);
    }
};

}  // namespace rr_util

RCLCPP_COMPONENTS_REGISTER_NODE(rr_util::JoystickDriver)

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rr_util::JoystickDriver>();

    RCLCPP_INFO(node->get_logger(), "joystick_driver node ready.");
    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "Shutting down joystick_driver node.");
    rclcpp::shutdown();
    return 0;
}
