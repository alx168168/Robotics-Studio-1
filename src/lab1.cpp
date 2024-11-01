#include <memory> //utilities for managing dynamic memory (e.g., smart pointers)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h> //Includes functionalities to convert between ROS 2 image messages and OpenCV images
#include <opencv2/imgproc/imgproc.hpp> //OpenCV functionalities for image processing and GUI.
#include <opencv2/highgui/highgui.hpp>

/**
 * @class ImageProcessor
 * @brief A ROS 2 node that subscribes to an image topic, processes the image, and republishes the processed image.
 */
class ImageProcessor : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the ImageProcessor node.
     * This initializes the node, subscribes to the `/camera/image_raw` topic, and creates a publisher
     * to publish the processed images on the `processed_image` topic.
     */
    ImageProcessor() : Node("image_processor") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&ImageProcessor::image_callback, this, std::placeholders::_1)); //subscribes to /camera/image_raw topic
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10); //publishes to processed_image topic
    }

private:
    /**
     * @brief Callback function that is triggered each time a new image is received.
     * This function processes the image, adds a green circle at the center, and republishes it.
     * @param msg The received image message from the `/camera/image_raw` topic.
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) { //called every time a new image message is received on the subscribed topic
        RCLCPP_INFO(this->get_logger(), "Receiving video frame"); //Logs that a video frame is received

        // Convert ROS2 image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Get the dimensions of the image
        int height = cv_ptr->image.rows;
        int width = cv_ptr->image.cols;

        // Draw a circle at the center of the image
        cv::Point center(width / 2, height / 2);
        int radius = 20;
        cv::Scalar color(0, 255, 0); // Green
        int thickness = 2;
        cv::circle(cv_ptr->image, center, radius, color, thickness);

        // Convert OpenCV image back to ROS Image message
        sensor_msgs::msg::Image::SharedPtr processed_image_msg = cv_ptr->toImageMsg();

        // Publish the modified image
        publisher_->publish(*processed_image_msg);
    }

    /**
     * @brief Subscription object for receiving image messages.
     * This subscribes to the `/camera/image_raw` topic.
     */
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    /**
     * @brief Publisher object for sending processed image messages.
     * This publishes processed images to the `processed_image` topic.
     */
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessor>());
    rclcpp::shutdown();
    return 0;
}
