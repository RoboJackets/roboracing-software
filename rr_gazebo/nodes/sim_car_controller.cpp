#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rr_msgs/msg/chassis_state.hpp>
#include <rr_msgs/msg/speed.hpp>
#include <rr_msgs/msg/steering.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

namespace rr_gazebo {

class PIDController {
  public:
    PIDController(double p, double i, double d) : P(p), I(i), D(d) {}

    void setDesired(double value) {
        desired = value;
    }

    double operator()(double current) {
        auto error = current - desired;
        accError += error;
        auto dError = error - lastError;
        auto ret = P * error + I * accError - D * dError;
        lastError = error;
        accError *= 0.999;
        return ret;
    }

  private:
    double P, I, D;
    double desired = 0.0;
    double lastError = 0.0;
    double accError = 0.0;
};

class SimCarController : public rclcpp::Node {
  public:
    explicit SimCarController() : rclcpp::Node("sim controller node") {
        // Declare params
        this->declare_parameter<double>("wheelbase");
        this->declare_parameter<double>("max_torque");
        this->declare_parameter<double>("wheel_radius_back");
        this->declare_parameter<double>("speed_kP");
        this->declare_parameter<double>("speed_kD");
        this->declare_parameter<double>("speed_kI");
        this->declare_parameter<std::string>("left_motor_joint_name");
        this->declare_parameter<std::string>("right_motor_joint_name");

        // Publishers

        left_drive_pub_ = create_publisher<std_msgs::msg::Float64>("/left_wheel_effort_controller/command",
                                                                   rclcpp::SystemDefaultsQoS());

        right_drive_pub_ = create_publisher<std_msgs::msg::Float64>("/right_wheel_effort_controller/command",
                                                                    rclcpp::SystemDefaultsQoS());

        left_steering_pub_ = create_publisher<std_msgs::msg::Float64>("/left_steer_position_controller/command",
                                                                      rclcpp::SystemDefaultsQoS());

        right_steering_pub_ = create_publisher<std_msgs::msg::Float64>("/right_steer_position_controller/command",
                                                                       rclcpp::SystemDefaultsQoS());

        chassis_state_pub_ =
              create_publisher<rr_msgs::msg::ChassisState>("/chassis_state", rclcpp::SystemDefaultsQoS());

        odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odometry_encoder", rclcpp::SystemDefaultsQoS());

        // Subscribers

        speed_sub_ = create_subscription<rr_msgs::msg::Speed>(
              "/speed", rclcpp::SystemDefaultsQoS(),
              std::bind(&rr_gazebo::SimCarController::speedCallback, this, std::placeholders::_1));

        steer_sub_ = create_subscription<rr_msgs::msg::Steering>(
              "/steering", rclcpp::SystemDefaultsQoS(),
              std::bind(&rr_gazebo::SimCarController::steeringCallback, this, std::placeholders::_1));

        state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
              "/joint_states", rclcpp::SystemDefaultsQoS(),
              std::bind(&rr_gazebo::SimCarController::jointStateCallback, this, std::placeholders::_1));
    }
    // Pubs and subs
    PIDController left_controller{ speed_kP, speed_kI, speed_kD };
    PIDController right_controller{ speed_kP, speed_kI, speed_kD };

    double speed_measured_left = 0.0;
    double speed_measured_right = 0.0;
    double max_torque = this->get_parameter("max_torque").as_double();
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_drive_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_drive_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_steering_pub_;
    rclcpp::Publisher<rr_msgs::msg::ChassisState>::SharedPtr chassis_state_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

    rclcpp::Subscription<rr_msgs::msg::Speed>::SharedPtr speed_sub_;
    rclcpp::Subscription<rr_msgs::msg::Steering>::SharedPtr steer_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;

    void fillWheelSpeeds(double& left_speed, double& right_speed) {
        if (steer_set_point == 0) {
            left_speed = speed_set_point;
            right_speed = speed_set_point;
        } else {
            double turning_radius = chassis_length / fabs(sin(steer_set_point));
            double radius_left = turning_radius - copysign(chassis_width_2, steer_set_point);
            double radius_right = turning_radius + copysign(chassis_width_2, steer_set_point);

            left_speed = speed_set_point * radius_left / turning_radius;
            right_speed = speed_set_point * radius_right / turning_radius;
        }
    }

    constexpr double get_steer_ang(double phi) {
        return (phi >= 0.0) ? (M_PI_2 - phi) : (-M_PI_2 - phi);
    }

    void fillSteeringPositions(double& left, double& right) {
        double center_y = chassis_length * tan((M_PI_2)-steer_set_point);
        left = get_steer_ang(atan(inv_chassis_length * (center_y - chassis_width_2)));
        right = get_steer_ang(atan(inv_chassis_length * (center_y + chassis_width_2)));
    }

    void speedCallback(const rr_msgs::msg::Speed::SharedPtr msg) {
        speed_set_point = -msg->speed;
    }

    void steeringCallback(const rr_msgs::msg::Steering::SharedPtr msg) {
        steer_set_point = -msg->angle;
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        auto iter = std::find(msg->name.begin(), msg->name.end(), left_motor_joint_name);

        if (iter != msg->name.end()) {
            auto index = std::distance(msg->name.begin(), iter);

            speed_measured_left = (-msg->velocity[index]) * (wheel_circumference / (2 * M_PI));
        }

        iter = std::find(msg->name.begin(), msg->name.end(), right_motor_joint_name);

        if (iter != msg->name.end()) {
            auto index = std::distance(msg->name.begin(), iter);

            speed_measured_right = (-msg->velocity[index]) * (wheel_circumference / (2 * M_PI));
        }
    }

  private:
    // Parameters

    double chassis_length = this->get_parameter("wheelbase").as_double();
    double chassis_width = this->get_parameter("track").as_double();
    double wheel_radius = this->get_parameter("wheel_radius_back").as_double();
    double speed_kP = this->get_parameter("speed_kP").as_double();
    double speed_kD = this->get_parameter("speed_kD").as_double();
    double speed_kI = this->get_parameter("speed_kI").as_double();
    std::string left_motor_joint_name = this->get_parameter("left_motor_joint_name").as_string();
    std::string right_motor_joint_name = this->get_parameter("right_motor_joing_name").as_string();

    double inv_chassis_length = 1.0 / chassis_length;
    double chassis_width_2 = chassis_width / 2.0;
    double wheel_circumference = 2.0 * M_PI * wheel_radius;

    double speed_set_point = 0.0;
    double steer_set_point = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rr_gazebo::SimCarController>();
    rclcpp::Rate rate(30);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        double left_speed, right_speed;
        node->fillWheelSpeeds(left_speed, right_speed);

        node->left_controller.setDesired(left_speed);
        node->right_controller.setDesired(right_speed);

        std_msgs::msg::Float64 left_drive_msg;
        auto set_torque = node->left_controller(node->speed_measured_left);
        left_drive_msg.data = std::max(-node->max_torque, std::min(set_torque, node->max_torque));
        node->left_drive_pub_->publish(left_drive_msg);

        std_msgs::msg::Float64 right_drive_msg;
        set_torque = node->right_controller(node->speed_measured_right);
        right_drive_msg.data = std::max(-node->max_torque, std::min(set_torque, node->max_torque));
        node->right_drive_pub_->publish(right_drive_msg);

        std_msgs::msg::Float64 left_steer_msg, right_steer_msg;

        node->fillSteeringPositions(left_steer_msg.data, right_steer_msg.data);

        node->left_steering_pub_->publish(left_steer_msg);
        node->right_steering_pub_->publish(right_steer_msg);

        rclcpp::Clock c;
        rr_msgs::msg::ChassisState chassis_state_msg;
        chassis_state_msg.header.stamp = c.now();
        chassis_state_msg.speed_mps = -(node->speed_measured_left + node->speed_measured_right) / 2.0;
        chassis_state_msg.mux_autonomous = true;
        chassis_state_msg.estop_on = false;
        node->chassis_state_pub_->publish(chassis_state_msg);

        // Pose and Twist Odometry Information for EKF localization
        geometry_msgs::msg::PoseWithCovariance pose_msg;
        geometry_msgs::msg::TwistWithCovariance twist_msg;

        nav_msgs::msg::Odometry odometry_msg;
        odometry_msg.header.stamp = c.now();
        odometry_msg.header.frame_id = "odom";
        odometry_msg.child_frame_id = "base_footprint";
        odometry_msg.twist.twist.linear.x = chassis_state_msg.speed_mps;
        odometry_msg.twist.twist.linear.y = 0.0;  // can't move sideways instantaneously
        // #TODO: set twist covariance?
        // #TODO: if need be, use steering for extra data
        // #see https://answers.ros.org/question/296112/odometry-message-for-ackerman-car/
        node->odometry_pub_->publish(odometry_msg);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 1;
}

}  // namespace rr_gazebo