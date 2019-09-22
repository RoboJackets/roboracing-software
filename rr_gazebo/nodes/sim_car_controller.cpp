#include <parameter_assertions/assertions.h>
#include <ros/ros.h>
#include <rr_msgs/chassis_state.h>
#include <rr_msgs/speed.h>
#include <rr_msgs/steering.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

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

double speed_set_point = 0.0;
double speed_measured_left = 0.0;
double speed_measured_right = 0.0;

double steer_set_point = 0.0;

double chassis_length;
double chassis_width;
double inv_chassis_length;
double chassis_width_2;
double max_torque;
double wheel_circumference;
std::string left_motor_joint_name;
std::string right_motor_joint_name;

void speedCallback(const rr_msgs::speedConstPtr &msg) {
    speed_set_point = -msg->speed;
}

void steeringCallback(const rr_msgs::steeringConstPtr &msg) {
    steer_set_point = -msg->angle;
}

void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg) {
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

constexpr double get_steer_ang(double phi) {
    return (phi >= 0.0) ? (M_PI_2 - phi) : (-M_PI_2 - phi);
}

void fillSteeringPositions(const double set_angle, double &left, double &right) {
    auto center_y = chassis_length * tan((M_PI_2)-set_angle);
    left = get_steer_ang(atan(inv_chassis_length * (center_y - chassis_width_2)));
    right = get_steer_ang(atan(inv_chassis_length * (center_y + chassis_width_2)));
}

void fillWheelSpeeds(const double steering_angle, const double speed, double &left, double &right) {
    if (steering_angle == 0.0) {
        left = speed;
        right = speed;
    } else {
        auto turning_radius = chassis_length / fabs(sin(steering_angle));
        auto radius_left = turning_radius - copysign(chassis_width_2, steering_angle);
        auto radius_right = turning_radius + copysign(chassis_width_2, steering_angle);
        left = speed * radius_left / turning_radius;
        right = speed * radius_right / turning_radius;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "macaroni_controller");

    ros::NodeHandle handle;
    ros::NodeHandle private_handle("~");

    assertions::getParam(private_handle, "wheelbase", chassis_length);
    assertions::getParam(private_handle, "track", chassis_width);
    assertions::getParam(private_handle, "max_torque", max_torque);
    double wheel_radius;
    assertions::getParam(private_handle, "wheel_radius_back", wheel_radius);
    double speed_kP;
    assertions::getParam(private_handle, "speed_kP", speed_kP);
    double speed_kD;
    assertions::getParam(private_handle, "speed_kD", speed_kD);
    double speed_kI;
    assertions::getParam(private_handle, "speed_kI", speed_kI);
    assertions::getParam(private_handle, "left_motor_joint_name", left_motor_joint_name);
    assertions::getParam(private_handle, "right_motor_joint_name", right_motor_joint_name);

    inv_chassis_length = 1.0 / chassis_length;
    chassis_width_2 = chassis_width / 2.0;
    wheel_circumference = 2.0 * M_PI * wheel_radius;

    auto leftDrivePublisher = handle.advertise<std_msgs::Float64>("/left_wheel_effort_controller/command", 1);
    auto rightDrivePublisher = handle.advertise<std_msgs::Float64>("/right_wheel_effort_controller/command", 1);
    auto leftSteeringPublisher = handle.advertise<std_msgs::Float64>("/left_steer_position_controller/command", 1);
    auto rightSteeringPublisher = handle.advertise<std_msgs::Float64>("/right_steer_position_controller/command", 1);
    auto chassisStatePublisher = handle.advertise<rr_msgs::chassis_state>("/chassis_state", 1);

    auto speedSub = handle.subscribe("/speed", 1, speedCallback);
    auto steerSub = handle.subscribe("/steering", 1, steeringCallback);
    auto stateSub = handle.subscribe("/joint_states", 1, jointStateCallback);

    PIDController left_controller{ speed_kP, speed_kI, speed_kD };
    PIDController right_controller{ speed_kP, speed_kI, speed_kD };

    ros::Rate rate{ 30 };
    while (ros::ok()) {
        ros::spinOnce();

        double left_speed, right_speed;
        fillWheelSpeeds(steer_set_point, speed_set_point, left_speed, right_speed);

        left_controller.setDesired(left_speed);
        right_controller.setDesired(right_speed);

        std_msgs::Float64 leftDriveMsg;
        auto set_torque = left_controller(speed_measured_left);
        leftDriveMsg.data = std::max(-max_torque, std::min(set_torque, max_torque));
        leftDrivePublisher.publish(leftDriveMsg);

        std_msgs::Float64 rightDriveMsg;
        set_torque = right_controller(speed_measured_right);
        rightDriveMsg.data = std::max(-max_torque, std::min(set_torque, max_torque));
        rightDrivePublisher.publish(rightDriveMsg);

        std_msgs::Float64 leftSteerMsg;
        std_msgs::Float64 rightSteerMsg;

        fillSteeringPositions(steer_set_point, leftSteerMsg.data, rightSteerMsg.data);

        leftSteeringPublisher.publish(leftSteerMsg);
        rightSteeringPublisher.publish(rightSteerMsg);

        rr_msgs::chassis_state chassisStateMsg;
        chassisStateMsg.header.stamp = ros::Time::now();
        chassisStateMsg.speed_mps = -(speed_measured_left + speed_measured_right) / 2;
        chassisStateMsg.mux_autonomous = true;
        chassisStateMsg.estop_on = false;
        chassisStatePublisher.publish(chassisStateMsg);

        rate.sleep();
    }

    return 0;
}
