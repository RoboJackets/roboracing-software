#include <gtest/gtest.h>
#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <sensor_msgs/Joy.h>

class JoystickDriverTestSuite : public testing::Test {
  public:
    JoystickDriverTestSuite()
          : handle()
          , handle_private("~")
          , joy_pub(handle.advertise<sensor_msgs::Joy>("/joy", 1))
          , speed_sub(handle.subscribe("/speed", 1, &JoystickDriverTestSuite::speedCallback, this))
          , steering_sub(handle.subscribe("/steering", 1, &JoystickDriverTestSuite::steeringCallback, this)) {
        handle_private.param("speed_max", speed_max, 0.0f);
        handle_private.param("angle_max", angle_max, 0.0f);
        handle_private.param("speed_axis", speed_axis, 0);
        handle_private.param("angle_axis", angle_axis, 0);
    }

    void speedCallback(const rr_platform::speed::ConstPtr& msg) {
        speed_msg = *msg;
        speed_received = true;
    }
    void steeringCallback(const rr_platform::steering::ConstPtr& msg) {
        steering_msg = *msg;
        steering_received = true;
    }

  protected:
    virtual void SetUp() override {
        while (!IsNodeReady()) {
            ros::spinOnce();
        }
    }

    virtual void TearDown() override {}

    bool IsNodeReady() {
        return (joy_pub.getNumSubscribers() > 0) && (speed_sub.getNumPublishers() > 0) &&
               (steering_sub.getNumPublishers() > 0);
    }

    ros::NodeHandle handle;
    ros::NodeHandle handle_private;
    ros::Publisher joy_pub;
    ros::Subscriber speed_sub;
    ros::Subscriber steering_sub;
    volatile bool speed_received = false;
    volatile bool steering_received = false;
    rr_platform::speed speed_msg;
    rr_platform::steering steering_msg;
    float angle_max;
    float speed_max;
    int angle_axis;
    int speed_axis;
};

TEST_F(JoystickDriverTestSuite, FullForward) {
    speed_received = false;
    steering_received = false;

    sensor_msgs::Joy joy_msg;
    joy_msg.axes = { 0, 0, 0, 0, 0, 0, 0, 0 };              // 8 axes
    joy_msg.buttons = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // 11 buttons
    joy_msg.axes[angle_axis] = 0.0;
    joy_msg.axes[speed_axis] = -1.0;
    joy_pub.publish(joy_msg);

    while (!speed_received || !steering_received) {
        ros::spinOnce();
    }

    EXPECT_FLOAT_EQ(speed_max, speed_msg.speed);
    EXPECT_FLOAT_EQ(0.0, steering_msg.angle);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_joystick_driver");
    testing::InitGoogleTest(&argc, argv);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}