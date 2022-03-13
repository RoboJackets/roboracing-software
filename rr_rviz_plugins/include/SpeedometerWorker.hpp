#ifndef SPEEDOMETER_WORKER_H
#define SPEEDOMETER_WORKER_H

#include <QObject>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <rr_msgs/msg/chassis_state.hpp>

class SpeedometerWorker : public QObject {
    Q_OBJECT
  public:
    SpeedometerWorker();
    ~SpeedometerWorker();

  public slots:
    void startNode();

  signals:
    void updateSpeed(float current_speed);
    void error(QString err);

  private:
    void speedCallback(const rr_msgs::msg::ChassisState::SharedPtr msg);

    // add your variables here
    float current_speed;
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Subscription<rr_msgs::msg::ChassisState>::SharedPtr speed_sub_;
};

#endif