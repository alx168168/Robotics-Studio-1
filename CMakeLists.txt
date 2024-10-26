cmake_minimum_required(VERSION 3.8)
project(lab)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(lab1 src/lab1.cpp)
ament_target_dependencies(lab1 rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(lab2 src/lab2.cpp)
ament_target_dependencies(lab2 rclcpp sensor_msgs std_msgs)

add_executable(lab4 src/lab4.cpp)
ament_target_dependencies(lab4 rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs)

add_executable(lab5 src/lab5.cpp)
ament_target_dependencies(lab5 rclcpp geometry_msgs sensor_msgs OpenCV)

add_executable(sprint2 src/sprint2.cpp)
ament_target_dependencies(sprint2 rclcpp nav_msgs sensor_msgs OpenCV)

add_executable(map_compare src/map_compare.cpp)
ament_target_dependencies(map_compare rclcpp OpenCV)

add_executable(cylinder_detector src/cylinder_detector.cpp)
ament_target_dependencies(cylinder_detector rclcpp sensor_msgs geometry_msgs visualization_msgs tf2_ros tf2_geometry_msgs nav_msgs OpenCV) 

add_executable(nav_to_goal src/nav_to_goal.cpp)
ament_target_dependencies(nav_to_goal rclcpp sensor_msgs geometry_msgs visualization_msgs tf2_ros tf2_geometry_msgs nav_msgs OpenCV nav2_msgs rclcpp_action) 

install(TARGETS
  lab1
  lab2
  lab4 
  lab5
  sprint2
  map_compare
  cylinder_detector
  nav_to_goal
  DESTINATION lib/${PROJECT_NAME})

ament_package()
