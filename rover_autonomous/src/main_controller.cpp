// navigation_controller.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <vector>
#include <optional>

#include "rover_autonomous/mission_parser.hpp"

class NavigationController : public rclcpp::Node
{
public:
    NavigationController() : Node("navigation_controller")
    {

        declare_parameter<std::string>("mission_file", "");
        std::string mission_file = get_parameter("mission_file").as_string();

        try
        {
            mission_points_ = MissionParser::parse_mission(mission_file);
            RCLCPP_INFO(get_logger(), "Mission chargée avec %zu points", mission_points_.size());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Erreur lors du chargement de la mission: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10,
            std::bind(&NavigationController::gps_callback, this, std::placeholders::_1));
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/navigation_cmd", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&NavigationController::control_loop, this));
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // Mise à jour position GPS et historique pour estimer cap
        if (last_position_)
        {
            prev_position_ = last_position_;
        }
        last_position_ = *msg;
    }

    // Distance haversine en mètres
    double haversine(double lat1, double lon1, double lat2, double lon2)
    {
        constexpr double R = 6371000.0;
        double dlat = (lat2 - lat1) * M_PI / 180.0;
        double dlon = (lon2 - lon1) * M_PI / 180.0;
        double a = sin(dlat / 2) * sin(dlat / 2) +
                   cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) * sin(dlon / 2) * sin(dlon / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        return R * c;
    }

    // Azimut (bearing) radians de nord, vers est positif
    double bearing(double lat1, double lon1, double lat2, double lon2)
    {
        double phi1 = lat1 * M_PI / 180.0;
        double phi2 = lat2 * M_PI / 180.0;
        double dlon = (lon2 - lon1) * M_PI / 180.0;
        double y = sin(dlon) * cos(phi2);
        double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(dlon);
        return atan2(y, x);
    }

    void control_loop()
    {
        if (!last_position_ || mission_points_.empty())
            return;
        if (current_waypoint_ >= mission_points_.size())
        {
            RCLCPP_INFO(get_logger(), "Mission terminée");
            publish_cmd(0, 0);
            timer_->cancel();
            return;
        }

        MissionPoint target = mission_points_[current_waypoint_];

        // Distance au waypoint
        double dist = haversine(last_position_->latitude, last_position_->longitude,
                                target.latitude, target.longitude);

        if (dist < target.tolerance)
        {
            RCLCPP_INFO(get_logger(), "Waypoint %zu atteint (dist=%.2fm)", current_waypoint_, dist);
            current_waypoint_++;
            return;
        }

        // Cap vers waypoint
        double target_bearing = bearing(last_position_->latitude, last_position_->longitude,
                                        target.latitude, target.longitude);

        // Estimation cap actuel par déplacement GPS (sinon 0)
        double current_heading = 0.0;
        if (prev_position_)
        {
            current_heading = bearing(prev_position_->latitude, prev_position_->longitude,
                                      last_position_->latitude, last_position_->longitude);
        }

        // Différence angulaire normalisée [-pi, pi]
        double angle_error = normalize_angle(target_bearing - current_heading);

        // Commande proportionnelle simple
        double linear_speed = std::min(dist * 0.5, 1.0); // max 1 m/s
        double angular_speed = angle_error * 1.0;        // gain angulaire

        publish_cmd(linear_speed, angular_speed);
    }

    void publish_cmd(double lin, double ang)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = lin;
        msg.angular.z = ang;
        // RCLCPP_INFO(get_logger(), "Publishing navigation command: linear=%.2f m/s angular=%.2f rad/s", lin, ang);
        cmd_pub_->publish(msg);
    }

    // Normalise angle en radians vers [-pi, pi]
    double normalize_angle(double angle)
    {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::optional<sensor_msgs::msg::NavSatFix> last_position_, prev_position_;
    std::vector<MissionPoint> mission_points_;
    size_t current_waypoint_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationController>());
    rclcpp::shutdown();
    return 0;
}
