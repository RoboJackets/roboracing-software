#ifndef WORKER_H
#define WORKER_H

#include <QObject>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <rr_msgs/msg/speed.hpp>

class Worker : public QObject {
    Q_OBJECT
  public:
    Worker();
    ~Worker();

  public slots:
    void startNode();

  signals:
    void finished(float current_speed);
    void error(QString err);

  private:
    void speedCallback(const rr_msgs::msg::Speed::SharedPtr msg);
    // add your variables here
    float current_speed;
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Publisher<rr_msgs::msg::Speed>::SharedPtr speed_pub_;
    rclcpp::Subscription<rr_msgs::msg::Speed>::SharedPtr speed_sub_;
};

#endif