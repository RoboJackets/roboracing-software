#include "image_flipper.h"

namespace rr_util {
void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImageConstPtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("CV-Bridge error: %s", e.what());
        return;
    }

    const cv::Mat &input = cv_ptr->image;
    cv::flip(input, output.image, flip_code);

    output.encoding = cv_ptr->encoding;
    img_pub.publish(output.toImageMsg());
}

image_flipper {
    auto nh = getNodeHandle();
    auto nhp = getPrivateNodeHandle();
    image_transport::ImageTransport it(nh);

    // This parameter is directly forwarded to OpenCV, so see OpenCV's
    // documentation for definition.
    // https://docs.opencv.org/2.4/modules/core/doc/operations_on_arrays.html#void
    // flip(InputArray src, OutputArray dst, int flipCode)
    nhp.getParam("flip_code", flip_code);

    img_sub = it.subscribe("image", 1, &image_flipper::imageCallback, this);
    img_pub = it.advertise("image_flipped", 1);
}
}  // namespace rr_util
PLUGINLIB_EXPORT_CLASS(rr_util::image_flipper, nodelet::Nodelet)
