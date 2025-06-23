#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <GeographicLib/Geodesic.hpp>

class NavigationNode : public rclcpp::Node {
public:
  NavigationNode() : Node("navigation_node") {
    // Abonnements
    waypoint_sub_ = create_subscription<mavros_msgs::msg::WaypointList>(
      "/mission_waypoints", 10,
      [this](const mavros_msgs::msg::WaypointList::SharedPtr msg) {
        waypoints_ = *msg;
      });
      
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>("/gps_fix", 10,
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        current_gps_ = *msg;
      });
      
    // Publication des commandes
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Timer de contrôle
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { navigate(); });
  }

private:

  // Variables membres à déclarer
  rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr waypoint_sub_;
  mavros_msgs::msg::WaypointList waypoints_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  sensor_msgs::msg::NavSatFix current_gps_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  size_t current_waypoint_index_ = 0;  // initialisation à 0

  void navigate() {
    if (waypoints_.waypoints.empty()) return;
    
    auto& target = waypoints_.waypoints[current_waypoint_index_];
    
    // Calcul distance/azimuth
    double distance, azimuth;
    GeographicLib::Geodesic::WGS84().Inverse(
      current_gps_.latitude, current_gps_.longitude,
      target.x_lat, target.y_long,
      distance, azimuth);
    
    // Logique de navigation
    if (distance < 1.0) { // Seuil d'arrivée
      current_waypoint_index_++;
    } else {
      auto cmd = calculate_velocity(distance, azimuth);
      cmd_vel_pub_->publish(cmd);
    }
  }
  
  geometry_msgs::msg::Twist calculate_velocity(double /*distance*/, double azimuth) {
    // Algorithme Pure Pursuit simplifié
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.5;  // Vitesse constante
    cmd.angular.z = azimuth * 0.5;  // Gain de direction
    return cmd;
  }

  // Variables membres...
};