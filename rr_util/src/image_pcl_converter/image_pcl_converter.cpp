/*
 * Borrows from old color_detector_avc and from image_transform.
 * N.B. pass in a binary image in 1-channel "mono8" format.
 * Any non-zero element in the input image becomes a point in a
 * cloud.
 */

#include <cv_bridge/cv_bridge.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <boost/algorithm/string.hpp>
#include <opencv2/opencv.hpp>


using uchar = unsigned char;
namespace rr_util
{
class ImagePclConverter : rclcpp::Node {    
public:
    explicit ImagePclConverter(const rclcpp::NodeOptions & options) : rclcpp::Node("image_pcl_converter", options)
    {
        std::string topicsConcat;
        this->declare_parameter<std::string>("image_topics", topicsConcat);
        this->declare_parameter<double>("px_per_meter", pxPerMeter);

        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

        std::vector<std::string> topics;
        boost::split(topics, topicsConcat, boost::is_any_of(" ,"));
        std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs;
        for (const std::string& topic : topics) {
            if (topic.empty())
                continue;

            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub = create_subscription<sensor_msgs::msg::Image>(
                                                topic, 
                                                rclcpp::SystemDefaultsQoS(), 
                                                std::bind(ImagePclConverter::transformedImageCB, this, std::placeholders::_1)
                                                );
            image_subs.push_back(sub);
            std::string subscribe_message = "image_pcl_converter subscribed to " + topic;
            RCLCPP_INFO(this->get_logger(), subscribe_message.data());
            std::string newTopic = topic + "_cloud";
            std::string publisher_message = "Creating new topic " + newTopic;
            RCLCPP_INFO(this->get_logger(), publisher_message.data());
            cloud_pubs[topic] = this->create_publisher<sensor_msgs::msg::PointCloud2>(newTopic, 1);
        }
    }

    private:
        void transformedImageCB(const sensor_msgs::msg::Image::SharedPtr msg, const std::string& topic) {
            if (cloud_pubs[topic]->get_subscription_count() == 0) {
                return;
            }

            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvShare(msg);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(get_logger(), "CV_Bridge error: %s", e.what());
                return;
            }

            const cv::Mat& in_image = cv_ptr->image;

            // get outline
            cv::Mat transformed;
            cv::Laplacian(in_image, transformed, CV_16SC1);

            cloud->clear();
            for (int r = 0; r < transformed.rows; r++) {
                auto* row = transformed.ptr<int16_t>(r);
                for (int c = 0; c < transformed.cols; c++) {
                    if (row[c] != 0) {
                        pcl::PointXYZ point;
                        point.y = static_cast<float>(((transformed.cols / 2.0f) - c) / pxPerMeter);
                        point.x = static_cast<float>((transformed.rows - r) / pxPerMeter);
                        point.z = 0.0;

                        cloud->push_back(point);
                    }
                }
            }

            pcl::PCLPointCloud2 cloud_pc2;
            pcl::toPCLPointCloud2(*cloud, cloud_pc2);
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl_conversions::fromPCL(cloud_pc2, cloud_msg);
            cloud_msg.header.frame_id = "base_footprint";
            cloud_msg.header.stamp = msg->header.stamp;
            cloud_pubs[topic]->publish(cloud_msg);
        }

    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> cloud_pubs;
    double pxPerMeter;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};
} //namespace rr_util

RCLCPP_COMPONENTS_REGISTER_NODE(rr_util::ImagePclConverter);

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rr_util::ImagePclConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
