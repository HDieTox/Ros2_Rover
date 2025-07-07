#include "rover_autonomous/navigation_controller.hpp"
#include <cmath>

NavigationController::NavigationController() : Node("navigation_controller") {
    // Configuration des paramètres
    declare_parameter("mission_file", "");
    std::string mission_file = get_parameter("mission_file").as_string();

    try {
        mission_points_ = MissionParser::parse_mission(mission_file);
        RCLCPP_INFO(get_logger(), "Mission chargée: %d points", mission_points_.size());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(get_logger(), "Erreur de chargement: %s", e.what());
        return;
    }

    // Initialisation TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Abonnements et publications
    gps_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/rtk_fix", 10,
        std::bind(&NavigationController::gps_callback, this, std::placeholders::_1));
    
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/navigation_cmd", 10);

    // Timer de contrôle (10Hz)
    mission_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&NavigationController::execute_mission, this));
}

void NavigationController::gps_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_ = msg;
}

void NavigationController::execute_mission() {
    if (!current_position_ || mission_points_.empty()) return;

    // Calcul distance et angle vers le waypoint
    MissionPoint target = mission_points_[current_waypoint_index_];
    double dx = target.latitude - current_position_->pose.pose.position.x;
    double dy = target.longitude - current_position_->pose.pose.position.y;
    double distance = std::hypot(dx, dy);

    // Vérification d'arrivée
    if (distance < target.tolerance) {
        if (++current_waypoint_index_ >= mission_points_.size()) {
            RCLCPP_INFO(get_logger(), "Mission terminée!");
            publish_navigation_command(0.0, 0.0);
            mission_timer_->cancel();
            return;
        }
    }

    // Calcul orientation (simplifié)
    double target_yaw = std::atan2(dy, dx);
    double current_yaw = 2 * std::atan2(
        current_position_->pose.pose.orientation.z,
        current_position_->pose.pose.orientation.w
    );
    double angle_error = target_yaw - current_yaw;

    // Commande proportionnelle
    double linear_speed = std::min(distance * 0.5, 1.0);
    double angular_speed = angle_error * 0.8;

    publish_navigation_command(linear_speed, angular_speed);
}

void NavigationController::publish_navigation_command(double linear, double angular) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    cmd_pub_->publish(cmd);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}