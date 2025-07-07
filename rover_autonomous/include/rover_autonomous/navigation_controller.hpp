#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "mission_parser.hpp"

class NavigationController : public rclcpp::Node {
public:
    NavigationController();

private:
    // Callbacks et fonctions internes
    void gps_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void execute_mission();
    void publish_navigation_command(double linear, double angular);
    
    // Variables membres
    std::vector<MissionPoint> mission_points_;
    size_t current_waypoint_index_ = 0;
    rclcpp::Time last_update_time_;
    nav_msgs::msg::Odometry::SharedPtr current_position_;
    
    // ROS
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr mission_timer_;
};