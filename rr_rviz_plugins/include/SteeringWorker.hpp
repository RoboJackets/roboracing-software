#ifndef STEERING_WORKER_H
#define STEERING_WORKER_H

#include <QObject>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <rr_msgs/msg/chassis_state.hpp>

class SteeringWorker : public QObject {
    Q_OBJECT
  public:
    SteeringWorker();
    ~SteeringWorker();

  public slots:
    void startNode();

  signals:
    void updateAngle(float current_angle);
    void error(QString err);

  private:
    void angleCallback(const rr_msgs::msg::ChassisState::SharedPtr msg);

    // add your variables here
    float current_angle;
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Subscription<rr_msgs::msg::ChassisState>::SharedPtr steering_sub_;
};

#endif