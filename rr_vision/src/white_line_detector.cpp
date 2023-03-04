#include "rr_vision/white_line_detector.hpp"

namespace white_line {
WhitePixelNode::WhitePixelNode(const rclcpp::NodeOptions & options) : rclcpp::Node("white_pixel_node", options)
{
    this->declare_parameter("min_white", 200);
    rclcpp::Parameter min_white_param = this->get_parameter("min_white");

    min_white = min_white_param.as_int();

    publisher_ = create_publisher<sensor_msgs::msg::Image>("white_pixels", 10);

    speed_publisher = this->create_publisher<rr_msgs::msg::Speed>("/speed", 10);
    steering_publisher = this->create_publisher<rr_msgs::msg::Steering>("/steering", 10);


    subscription_ = create_subscription<sensor_msgs::msg::Image>(
    "camera/color/image_raw", 10,
    std::bind(&WhitePixelNode::process_image, this, std::placeholders::_1));
}


void WhitePixelNode::process_image(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
    // Convert the ROS Image message to a cv::Mat
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat cv_image = cv_ptr->image;
    
    // Convert the image to grayscale
    cv::Mat gray_image;
    cv::cvtColor(cv_image, gray_image, cv::COLOR_BGR2GRAY);
    
    // Find the white pixels in the image
    cv::Mat white_pixels_mask = (gray_image > min_white);

    cv::Mat copy;
    white_pixels_mask.copyTo(copy);
    cv::Mat white_pixels_mask_left = copy(cv::Range(0, 1080), cv::Range(480, 960));

    white_pixels_mask.copyTo(copy);
    cv::Mat white_pixels_mask_right = copy(cv::Range(0, 1080), cv::Range(960, 1440));

    std::vector<cv::Point> white_pixels_left;
    std::vector<cv::Point> white_pixels_right;
    cv::findNonZero(white_pixels_mask_left, white_pixels_left);
    cv::findNonZero(white_pixels_mask_right, white_pixels_right);

    bool detect_left =  white_pixels_left.size() > 10;
    bool detect_right = white_pixels_right.size() > 10;

    rr_msgs::msg::Speed sp_cmd;
    rr_msgs::msg::Steering st_cmd;

    if (detect_left && !detect_right) {
        sp_cmd.speed = 0.5;
        st_cmd.angle = 0.3;
    } else if (!detect_left && detect_right){
        sp_cmd.speed = 0.5;
        st_cmd.angle = -0.3;
    } else {
        sp_cmd.speed = 1;
        st_cmd.angle = 0;
    }

    speed_publisher->publish(sp_cmd);
    steering_publisher->publish(st_cmd);
    // Highlight the white pixels in the image
    // Convert the output image to a ROS Image message and publish it
    cv_ptr->image = white_pixels_mask;
    cv_ptr->encoding = "mono8";
    sensor_msgs::msg::Image outmsg;
    cv_ptr->toImageMsg(outmsg);
    publisher_->publish(outmsg);
  }

  catch (const cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Error converting image message: %s", e.what());
    return;
  }
}

} //namespace white_line

RCLCPP_COMPONENTS_REGISTER_NODE(white_line::WhitePixelNode)
