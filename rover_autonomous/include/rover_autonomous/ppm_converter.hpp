#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class PPMConverter : public rclcpp::Node {
public:
    PPMConverter();

private:
    void navigation_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ppm_pub_;
};