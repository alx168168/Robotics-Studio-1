#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

/**
 * @class LaserScanProcessor
 * @brief A ROS 2 node that processes LaserScan messages, extracts subsets, and publishes them on different topics.
 */
class LaserScanProcessor : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the LaserScanProcessor node.
     * This initializes the node, subscribes to the `/scan` topic, and creates publishers for laser scan subsets.
     */
    LaserScanProcessor()
        : Node("laser_scan_processor")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanProcessor::scanCallback, this, std::placeholders::_1));

        scan_pub_1_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_subset_1", 10);
        scan_pub_2_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_subset_2", 10);
        scan_pub_nth_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_subset_nth", 10);
    }

private:
    /**
     * @brief Callback function that processes LaserScan messages and publishes subsets of the scan data.
     * This function extracts two angular ranges and publishes the corresponding laser scan data.
     * It also samples every nth point from the laser scan and publishes the subset.
     * @param scan The received LaserScan message.
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Logging the scan angles and increments
        RCLCPP_INFO(this->get_logger(), "angle_min: %f radians (%f degrees)", scan->angle_min, scan->angle_min * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "angle_max: %f radians (%f degrees)", scan->angle_max, scan->angle_max * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "angle_increment: %f radians (%f degrees)", scan->angle_increment, scan->angle_increment * 180.0 / M_PI);

        // Define angular ranges for the subsets in degrees
        double angle_min_deg_1 = 0.0;
        double angle_max_deg_1 = 30.0;
        double angle_min_deg_2 = 330.0;
        double angle_max_deg_2 = 360.0;

        // Convert angles from degrees to radians
        double angle_min_rad_1 = angle_min_deg_1 * M_PI / 180.0;
        double angle_max_rad_1 = angle_max_deg_1 * M_PI / 180.0;
        double angle_min_rad_2 = angle_min_deg_2 * M_PI / 180.0;
        double angle_max_rad_2 = angle_max_deg_2 * M_PI / 180.0;

        // Calculate the index range for subset 1 and 2 based on the angle increments
        int start_index_1 = static_cast<int>((angle_min_rad_1 - scan->angle_min) / scan->angle_increment);
        int end_index_1 = static_cast<int>((angle_max_rad_1 - scan->angle_min) / scan->angle_increment);
        int start_index_2 = static_cast<int>((angle_min_rad_2 - scan->angle_min) / scan->angle_increment);
        int end_index_2 = static_cast<int>((angle_max_rad_2 - scan->angle_min) / scan->angle_increment);

        // Create and publish the first laser scan subset
        auto subset_scan_1 = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
        subset_scan_1->ranges = std::vector<float>(scan->ranges.begin() + start_index_1, scan->ranges.begin() + end_index_1 + 1);
        subset_scan_1->angle_min = scan->angle_min + start_index_1 * scan->angle_increment;
        subset_scan_1->angle_max = scan->angle_min + end_index_1 * scan->angle_increment;

        // Create and publish the second laser scan subset
        auto subset_scan_2 = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
        subset_scan_2->ranges = std::vector<float>(scan->ranges.begin() + start_index_2, scan->ranges.begin() + end_index_2 + 1);
        subset_scan_2->angle_min = scan->angle_min + start_index_2 * scan->angle_increment;
        subset_scan_2->angle_max = scan->angle_min + end_index_2 * scan->angle_increment;

        scan_pub_1_->publish(*subset_scan_1);
        scan_pub_2_->publish(*subset_scan_2);

        // Create and publish the nth-point laser scan subset
        int n = 5; // Change this value to control the sampling interval
        auto subset_scan_nth = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
        subset_scan_nth->ranges.clear();

        // Sample every nth point from the laser scan data
        for (size_t i = 0; i < scan->ranges.size(); i += n)
        {
            subset_scan_nth->ranges.push_back(scan->ranges[i]);
        }

        // Adjust the angle increment for the nth-point subset
        subset_scan_nth->angle_increment = scan->angle_increment * n;
        subset_scan_nth->angle_max = subset_scan_nth->angle_min + subset_scan_nth->ranges.size() * subset_scan_nth->angle_increment;

        scan_pub_nth_->publish(*subset_scan_nth);
    }

    /**
     * @brief Subscription object for receiving LaserScan messages.
     * This subscribes to the `/scan` topic.
     */
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    /**
     * @brief Publisher object for publishing the first subset of laser scan data.
     * This publishes data to the `/scan_subset_1` topic.
     */
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_1_;

    /**
     * @brief Publisher object for publishing the second subset of laser scan data.
     * This publishes data to the `/scan_subset_2` topic.
     */
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_2_;

    /**
     * @brief Publisher object for publishing the nth-point subset of laser scan data.
     * This publishes data to the `/scan_subset_nth` topic.
     */
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_nth_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanProcessor>());
    rclcpp::shutdown();
    return 0;
}
 
