#include "rover_autonomous/ppm_converter.hpp"
#include <std_msgs/msg/string.hpp>

PPMConverter::PPMConverter() : Node("ppm_converter") {
    // Abonnement aux commandes de navigation
    nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/navigation_cmd", 10,
        std::bind(&PPMConverter::navigation_callback, this, std::placeholders::_1));
    
    // Publication des commandes PPM
    ppm_pub_ = create_publisher<std_msgs::msg::String>("/rover_commands", 10);
}

void PPMConverter::navigation_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std_msgs::msg::String ppm_cmd;
    ppm_cmd.data = "PPM Linear: " + std::to_string(msg->linear.x) + 
                   ", Angular: " + std::to_string(msg->angular.z);
    ppm_pub_->publish(ppm_cmd);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PPMConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}