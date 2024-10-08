#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
// #include <opencv2/xfeatures2d.hpp>

/**
 * @class MapCompareNode
 * @brief A ROS2 node for comparing and overlaying two occupancy grid maps using feature matching.
 */
class MapCompareNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructs the MapCompareNode and initiates the map overlay process.
     */
    MapCompareNode() : Node("map_compare_node")
    {
        // Paths to your maps
        std::string map1_path = "/home/student/ros2_ws/src/lab/maps/cartographer_map.pgm";
        std::string map2_path = "/home/student/ros2_ws/src/lab/maps/final_world.pgm";
        // std::string map1_path = "/home/student/ros2_ws/src/lab/maps/cart_world_final.pgm";
        // std::string map2_path = "/home/student/ros2_ws/src/lab/maps/world_final.pgm";
        overlayMaps(map1_path, map2_path);
    }

private:
    /**
     * @brief Overlays two maps by aligning them and combining their visuals.
     * 
     * This function loads the two specified map images, resizes the second map to match the first,
     * detects features in both maps, matches these features, and computes the homography
     * transformation to align the second map with the first. It then creates an overlay
     * image and saves it.
     * 
     * @param map1_path The file path to the first map image.
     * @param map2_path The file path to the second map image.
     */
    void overlayMaps(const std::string& map1_path, const std::string& map2_path)
    {
        // Load the two maps
        cv::Mat map1 = cv::imread(map1_path, cv::IMREAD_GRAYSCALE);
        cv::Mat map2 = cv::imread(map2_path, cv::IMREAD_GRAYSCALE);

        if (map1.empty() || map2.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Error loading the maps!");
            return;
        }

        // Scale both maps to the same size
        cv::Size size(map1.cols, map1.rows);
        cv::Mat map2_resized;
        cv::resize(map2, map2_resized, size);

        // Feature detection and matching for alignment
        cv::Ptr<cv::ORB> detector = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        detector->detectAndCompute(map1, cv::noArray(), keypoints1, descriptors1);
        detector->detectAndCompute(map2_resized, cv::noArray(), keypoints2, descriptors2);

        // Match features using FLANN matcher
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher->match(descriptors1, descriptors2, matches);

        // Filter out good matches
        double max_dist = 0; double min_dist = 100;
        for (int i = 0; i < descriptors1.rows; i++) {
            double dist = matches[i].distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }
        std::vector<cv::DMatch> good_matches;
        for (int i = 0; i < descriptors1.rows; i++) {
            if (matches[i].distance <= std::max(2 * min_dist, 30.0)) {
                good_matches.push_back(matches[i]);
            }
        }

        // Extract matched keypoints
        std::vector<cv::Point2f> points1, points2;
        for (size_t i = 0; i < good_matches.size(); i++) {
            points1.push_back(keypoints1[good_matches[i].queryIdx].pt);
            points2.push_back(keypoints2[good_matches[i].trainIdx].pt);
        }

        // Find the transformation matrix (homography)
        cv::Mat H = cv::findHomography(points2, points1, cv::RANSAC);

        // Warp the second map using the homography matrix to align with the first map
        cv::Mat map2_aligned;
        cv::warpPerspective(map2_resized, map2_aligned, H, map1.size());

        // Overlay the maps
        cv::Mat overlay;
        cv::addWeighted(map1, 0.5, map2_aligned, 0.5, 0, overlay);

        // Save and display the overlay
        cv::imwrite("/home/student/ros2_ws/src/lab/maps/overlaid_map1.pgm", overlay);
        cv::imwrite("/home/student/ros2_ws/src/lab/maps/overlaid_map1.png", overlay);
        // cv::imwrite("/home/student/ros2_ws/src/lab/maps/overlaid_map2.pgm", overlay);
        // cv::imwrite("/home/student/ros2_ws/src/lab/maps/overlaid_map2.png", overlay);
        cv::imshow("Overlayed Map", overlay);
        cv::waitKey(0);  // Wait for user to press any key

        RCLCPP_INFO(this->get_logger(), "Overlayed map saved and displayed.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapCompareNode>());
    rclcpp::shutdown();
    return 0;
}

