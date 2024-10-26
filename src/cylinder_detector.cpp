#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>

/**
 * @brief CylinderDetector class that handles detection of 30cm diameter cylinders from laser scan data
 *        and draws them on an occupancy grid map using OpenCV for image processing.
 */
class CylinderDetector : public rclcpp::Node
{
public:
    CylinderDetector() 
        : Node("cylinder_detector"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
    {
        laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderDetector::scanCallback, this, std::placeholders::_1));
        
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&CylinderDetector::mapCallback, this, std::placeholders::_1));
        
        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("modified_map", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&CylinderDetector::getTransform, this));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::msg::TransformStamped transform_stamped_;

    nav_msgs::msg::OccupancyGrid modified_map_;

    void getTransform()
    {
        try
        {
            transform_stamped_ = tf_buffer_.lookupTransform(
                "map", "base_scan", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform 'base_scan' to 'map': %s", ex.what());
        }
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (modified_map_.data.size() != msg->data.size())
        {
            modified_map_ = *msg;
        }
        modified_map_.info = msg->info;
        modified_map_.header = msg->header;

        for (size_t i = 0; i < msg->data.size(); i++)
        {
            if (modified_map_.data.at(i) < 100)
            {
                modified_map_.data.at(i) = msg->data.at(i);
            }
        }

        map_publisher_->publish(modified_map_);
    }

    void drawCircle(std::vector<int8_t> &data, int width, int center_x, int center_y, int radius)
    {
        int x = radius;
        int y = 0;
        int decision = 1 - radius;
        setCirclePoints(data, width, center_x, center_y, x, y);

        while (y < x)
        {
            y++;
            if (decision < 0)
            {
                decision += 2 * y + 1;
            }
            else
            {
                x--;
                decision += 2 * (y - x) + 1;
            }
            setCirclePoints(data, width, center_x, center_y, y, x);
        }
    }

    void setCirclePoints(std::vector<int8_t> &data, int width, int center_x, int center_y, int x, int y)
    {
        std::vector<std::pair<int, int>> points = {
            {center_x + x, center_y + y}, {center_x - x, center_y + y}, 
            {center_x + x, center_y - y}, {center_x - x, center_y - y}, 
            {center_x + y, center_y + x}, {center_x - y, center_y + x}, 
            {center_x + y, center_y - x}, {center_x - y, center_y - x}
        };

        for (const auto &[px, py] : points)
        {
            if (px >= 0 && px < width && py >= 0 && py < width)
            {
                data[py * width + px] = 100; // Mark as occupied
            }
        }
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        // Segment laser scan data into clusters
        auto clusters = segmentLaserScanData(scan_msg);

        for (const auto& cluster : clusters)
        {
            std::vector<cv::Vec3f> circles = detectCirclesInCluster(cluster);

            for (const auto& circle : circles)
            {
                double real_x, real_y;
                convertImageToCoordinates(circle[0], circle[1], scan_msg->range_max, 500, real_x, real_y);
                geometry_msgs::msg::Point pt = transformToGlobalFrame(real_x, real_y, transform_stamped_.transform);
                
                RCLCPP_INFO(this->get_logger(), "Cylinder detected at: x = %.2f, y = %.2f, radius = %.2f", pt.x, pt.y, circle[2]);

                // Convert circle to grid cells and draw on map
                float real_radius = circle[2] * (0.15 / 10);
                drawCircleOnMap(pt.x, pt.y, real_radius);
            }
        }
    }

    std::vector<std::vector<cv::Point>> segmentLaserScanData(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        std::vector<std::vector<cv::Point>> clusters;
        std::vector<cv::Point> current_cluster;
        double threshold = 0.05; // Distance threshold in meters
        double prev_range = scan_msg->ranges[0];
        double angle = scan_msg->angle_min;
        double angle_increment = scan_msg->angle_increment;

        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            double range = scan_msg->ranges[i];
            if (range < scan_msg->range_min || range > scan_msg->range_max)
            {
                continue; // Skip invalid ranges
            }

            // Convert range to x,y coordinates
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);
            cv::Point point(x * 100, y * 100); // Scale for image size

            if (current_cluster.empty() || std::abs(range - prev_range) < threshold)
            {
                current_cluster.push_back(point);
            }
            else
            {
                if (!current_cluster.empty())
                {
                    clusters.push_back(current_cluster);
                    current_cluster.clear();
                }
                current_cluster.push_back(point);
            }

            prev_range = range;
            angle += angle_increment;
        }

        if (!current_cluster.empty())
        {
            clusters.push_back(current_cluster);
        }

        return clusters;
    }

    std::vector<cv::Vec3f> detectCirclesInCluster(const std::vector<cv::Point> &cluster)
    {
        cv::Mat image(500, 500, CV_8UC1, cv::Scalar(0)); // Create blank image
        for (const auto &pt : cluster)
        {
            if (pt.x >= 0 && pt.x < 500 && pt.y >= 0 && pt.y < 500)
            {
                image.at<uchar>(pt.y, pt.x) = 255; // Mark the point
            }
        }

        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(image, circles, cv::HOUGH_GRADIENT, 1, 50, 50, 30, 7, 15);
        return circles;
    }

    void convertImageToCoordinates(int img_x, int img_y, double max_range, int image_size, double &real_x, double &real_y)
    {
        double scale = (2.0 * max_range) / image_size;
        real_x = (img_x - image_size / 2) * scale;
        real_y = (img_y - image_size / 2) * scale;
    }

    geometry_msgs::msg::Point transformToGlobalFrame(double x_local, double y_local, const geometry_msgs::msg::Transform &transform)
    {
        geometry_msgs::msg::Point pt;
        double theta = quaternionToYaw(transform.rotation);
        pt.x = transform.translation.x + (x_local * std::cos(theta) - y_local * std::sin(theta));
        pt.y = transform.translation.y + (x_local * std::sin(theta) + y_local * std::cos(theta));
        return pt;
    }

    double quaternionToYaw(const geometry_msgs::msg::Quaternion &q)
    {
        return std::atan2(2.0 * (q.z * q.w + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    void drawCircleOnMap(double x, double y, double radius)
    {
        int width = modified_map_.info.width;
        int height = modified_map_.info.height;
        int center_x = static_cast<int>((x + modified_map_.info.origin.position.x) / modified_map_.info.resolution);
        int center_y = static_cast<int>((y + modified_map_.info.origin.position.y) / modified_map_.info.resolution);
        
        if (center_x >= 0 && center_x < width && center_y >= 0 && center_y < height)
        {
            drawCircle(modified_map_.data, width, center_x, center_y, static_cast<int>(radius / modified_map_.info.resolution));
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CylinderDetector>());
    rclcpp::shutdown();
    return 0;
}
