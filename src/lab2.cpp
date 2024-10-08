#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LaserScanProcessor : public rclcpp::Node
{
public:
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
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        RCLCPP_INFO(this->get_logger(), "angle_min: %f radians (%f degrees)", scan->angle_min, scan->angle_min * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "angle_max: %f radians (%f degrees)", scan->angle_max, scan->angle_max * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "angle_increment: %f radians (%f degrees)", scan->angle_increment, scan->angle_increment * 180.0 / M_PI);

        double angle_min_deg_1 = 0.0;
        double angle_max_deg_1 = 30.0;
        double angle_min_deg_2 = 330.0;
        double angle_max_deg_2 = 360.0;

        double angle_min_rad_1 = angle_min_deg_1 * M_PI / 180.0;
        double angle_max_rad_1 = angle_max_deg_1 * M_PI / 180.0;
        double angle_min_rad_2 = angle_min_deg_2 * M_PI / 180.0;
        double angle_max_rad_2 = angle_max_deg_2 * M_PI / 180.0;

        int start_index_1 = static_cast<int>((angle_min_rad_1 - scan->angle_min) / scan->angle_increment);
        int end_index_1 = static_cast<int>((angle_max_rad_1 - scan->angle_min) / scan->angle_increment);
        int start_index_2 = static_cast<int>((angle_min_rad_2 - scan->angle_min) / scan->angle_increment);
        int end_index_2 = static_cast<int>((angle_max_rad_2 - scan->angle_min) / scan->angle_increment);

        auto subset_scan_1 = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
        subset_scan_1->ranges = std::vector<float>(scan->ranges.begin() + start_index_1, scan->ranges.begin() + end_index_1 + 1);
        subset_scan_1->angle_min = scan->angle_min + start_index_1 * scan->angle_increment;
        subset_scan_1->angle_max = scan->angle_min + end_index_1 * scan->angle_increment;

        auto subset_scan_2 = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
        subset_scan_2->ranges = std::vector<float>(scan->ranges.begin() + start_index_2, scan->ranges.begin() + end_index_2 + 1);
        subset_scan_2->angle_min = scan->angle_min + start_index_2 * scan->angle_increment;
        subset_scan_2->angle_max = scan->angle_min + end_index_2 * scan->angle_increment;

        scan_pub_1_->publish(*subset_scan_1);
        scan_pub_2_->publish(*subset_scan_2);

        // New nth-point subset code
        int n = 5; // Change this value to control the sampling interval
        auto subset_scan_nth = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
        subset_scan_nth->ranges.clear();

        for (size_t i = 0; i < scan->ranges.size(); i += n)
        {
            subset_scan_nth->ranges.push_back(scan->ranges[i]);
        }

        subset_scan_nth->angle_increment = scan->angle_increment * n;
        subset_scan_nth->angle_max = subset_scan_nth->angle_min + subset_scan_nth->ranges.size() * subset_scan_nth->angle_increment;

        scan_pub_nth_->publish(*subset_scan_nth);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_1_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_2_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_nth_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanProcessor>());
    rclcpp::shutdown();
    return 0;
}
 
