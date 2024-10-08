#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <geometry_msgs/msg/twist.hpp>

class ScanMatchingLocalizer : public rclcpp::Node {
public:
    ScanMatchingLocalizer() : Node("scan_matching_localizer"), angle_difference_(0.0) {
        map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&ScanMatchingLocalizer::mapCallback, this, std::placeholders::_1));

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanMatchingLocalizer::scanCallback, this, std::placeholders::_1));

        odom_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        cv::namedWindow("Map Section", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Laser Scan", cv::WINDOW_AUTOSIZE);
    }

private:
    cv::Mat map_image_, map_edges_;
    double map_scale_, origin_x_, origin_y_;
    int map_size_x_, map_size_y_;
    bool map_received_ = false;

    cv::Mat laser_image_, map_section_;
    bool first_scan_ = false;

    double angle_difference_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg) {
        map_image_ = occupancyGridToImage(mapMsg);
        map_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Map received and processed.");
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scanMsg) {
        if (!map_received_) return;

        // Convert laser scan to image (Image C)
        laser_image_ = laserScanToMat(scanMsg);

        // Assume robot position is known (for now hardcode a position)
        double robot_x = 1.0, robot_y = 1.0; // Replace with actual robot position

        // Extract map section around the robot (Image A)
        map_section_ = extractMapSection(robot_x, robot_y, map_scale_, origin_x_, origin_y_, map_image_, 100);

        // Detect edges (Image B)
        map_edges_ = extractEdges(map_section_);

        // Perform scan matching (compare Image B with Image C)
        if (first_scan_) {
            calculateYawChange(map_edges_, laser_image_);
        }

        // Update the robot's position
        propagateRobot();

        first_scan_ = true;
    }

    cv::Mat extractMapSection(double robot_x, double robot_y, double map_scale, double origin_x, double origin_y, cv::Mat& map_image, int section_size) {
        int robot_map_x = static_cast<int>((robot_x - origin_x) / map_scale);
        int robot_map_y = static_cast<int>((robot_y - origin_y) / map_scale);

        // Extract a section of the map centered on the robot's position
        int half_size = section_size / 2;
        int start_x = std::max(0, robot_map_x - half_size);
        int start_y = std::max(0, robot_map_y - half_size);
        int end_x = std::min(map_image.cols, robot_map_x + half_size);
        int end_y = std::min(map_image.rows, robot_map_y + half_size);

        return map_image(cv::Rect(start_x, start_y, end_x - start_x, end_y - start_y)).clone();
    }

    cv::Mat extractEdges(cv::Mat& input_image) {
        cv::Mat edges;
        cv::Canny(input_image, edges, 50, 150);
        return edges;
    }

    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        int img_size = 500;
        float max_range = scan->range_max;
        cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float range = scan->ranges[i];
            if (range > scan->range_min && range < scan->range_max) {
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
                int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;
                if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                    image.at<uchar>(y, x) = 255;
                }
            }
        }
        return image;
    }

    void calculateYawChange(cv::Mat& map_edges, cv::Mat& laser_image) {
        // Your feature matching logic goes here (same as from previous lab)
        // This function will calculate the yaw difference between the map edges and the laser scan.
    }

    void propagateRobot() {
        // Move robot to a new location using odometry (simplified for now)
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.angular.z = 0.5;  // Rotate a bit
        odom_publisher_->publish(twist_msg);
    }

    cv::Mat occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid) {
        int grid_data;
        unsigned int row, col, val;

        cv::Mat temp_img = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC1);

        for (row = 0; row < grid->info.height; row++) {
            for (col = 0; col < grid->info.width; col++) {
                grid_data = grid->data[row * grid->info.width + col];
                if (grid_data != -1) {
                    val = 255 - (255 * grid_data) / 100;
                    val = (val == 0) ? 255 : 0;
                    temp_img.at<uchar>(grid->info.height - row - 1, col) = val;
                } else {
                    temp_img.at<uchar>(grid->info.height - row - 1, col) = 0;
                }
            }
        }

        map_scale_ = grid->info.resolution;
        origin_x_ = grid->info.origin.position.x;
        origin_y_ = grid->info.origin.position.y;
        map_size_x_ = grid->info.width;
        map_size_y_ = grid->info.height;

        return temp_img;
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr odom_publisher_;

    double relative_orientaion_ = 0.0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanMatchingLocalizer>());
    rclcpp::shutdown();
    return 0;
}
