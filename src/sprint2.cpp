#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <geometry_msgs/msg/twist.hpp>

/**
 * @class ScanMatchingLocalizer
 * @brief A class that implements scan matching localization for a robot using 
 *        occupancy grid maps and laser scans.
 * 
 * This class subscribes to an occupancy grid map and laser scan data to perform 
 * scan matching, enabling the robot to determine its position within the map. 
 * It processes incoming data, extracts relevant sections from the map, detects 
 * edges, and calculates the robot's orientation based on the laser scans. 
 * 
 * The class also publishes velocity commands to control the robot's movement.
 */
class ScanMatchingLocalizer : public rclcpp::Node {
public:
    /**
     * @brief Constructs a ScanMatchingLocalizer object.
     * 
     * Initializes the node, sets up subscribers for the occupancy grid and 
     * laser scan topics, and creates a publisher for velocity commands. 
     * Additionally, it initializes OpenCV windows for displaying the map and 
     * laser scan images.
     */
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
    /**
     * @brief Callback function for processing incoming occupancy grid messages.
     * 
     * @param mapMsg A shared pointer to the occupancy grid message received.
     * This function converts the occupancy grid to an image and marks that 
     * the map has been received.
     */
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg) {
        map_image_ = occupancyGridToImage(mapMsg);
        map_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Map received and processed.");
    }

    /**
     * @brief Callback function for processing incoming laser scan messages.
     * 
     * @param scanMsg A shared pointer to the laser scan message received.
     * This function converts the laser scan data to an image, extracts a 
     * section of the map around the robot, detects edges in the map section, 
     * and updates the robot's position using scan matching.
     */
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

    /**
     * @brief Extracts a section of the map centered on the robot's position.
     * 
     * @param robot_x The x-coordinate of the robot's position.
     * @param robot_y The y-coordinate of the robot's position.
     * @param map_scale The scale of the map.
     * @param origin_x The x-coordinate of the map's origin.
     * @param origin_y The y-coordinate of the map's origin.
     * @param map_image The map image to extract from.
     * @param section_size The size of the section to extract.
     * @return A cv::Mat containing the extracted map section.
     */
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

    /**
     * @brief Detects edges in the given input image.
     * 
     * @param input_image The input image in which to detect edges.
     * @return A cv::Mat containing the detected edges.
     */
    cv::Mat extractEdges(cv::Mat& input_image) {
        cv::Mat edges;
        cv::Canny(input_image, edges, 50, 150);
        return edges;
    }

    /**
     * @brief Converts a laser scan message to a cv::Mat image.
     * 
     * @param scan A shared pointer to the laser scan message.
     * @return A cv::Mat representing the laser scan as an image.
     */
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

    /**
     * @brief Calculates the yaw change based on the edges of the map 
     *        and the laser scan image.
     * 
     * This function implements feature matching logic to calculate the yaw 
     * difference between the detected edges in the map and the laser scan.
     * 
     * @param map_edges The edges of the map extracted from the map section.
     * @param laser_image The image generated from the laser scan data.
     */
    void calculateYawChange(cv::Mat& map_edges, cv::Mat& laser_image) {
        // Your feature matching logic goes here (same as from previous lab)
        // This function will calculate the yaw difference between the map edges and the laser scan.
    }

    /**
     * @brief Propagates the robot's position using simplified odometry.
     * 
     * This function publishes velocity commands to move the robot based on 
     * the calculated orientation.
     */
    void propagateRobot() {
        // Move robot to a new location using odometry (simplified for now)
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.angular.z = 0.5;  // Rotate a bit
        odom_publisher_->publish(twist_msg);
    }

    /**
     * @brief Converts an occupancy grid message to a cv::Mat image.
     * 
     * @param grid A shared pointer to the occupancy grid message.
     * @return A cv::Mat representation of the occupancy grid.
     */
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

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_; //!< Subscriber for the occupancy grid map
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_; //!< Subscriber for the laser scan data
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr odom_publisher_; //!< Publisher for velocity commands

    double relative_orientaion_ = 0.0; //!< Variable to keep track of relative orientation

    cv::Mat map_image_, map_edges_; //!< Images for the map and edges
    double map_scale_, origin_x_, origin_y_; //!< Map scaling and origin coordinates
    int map_size_x_, map_size_y_; //!< Size of the map
    bool map_received_ = false; //!< Flag indicating whether the map has been received

    cv::Mat laser_image_, map_section_; //!< Images for the laser scan and extracted map section
    bool first_scan_ = false; //!< Flag indicating whether this is the first scan

    double angle_difference_; //!< Variable to store the angle difference
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanMatchingLocalizer>());
    rclcpp::shutdown();
    return 0;
}
