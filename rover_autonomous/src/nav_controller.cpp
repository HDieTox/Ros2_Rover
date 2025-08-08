#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <cmath>
#include <vector>
#include <optional>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using json = nlohmann::json;

constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double EARTH_RADIUS_M = 6371000.0;

std::pair<double, double> latlon_to_xy(double lat, double lon, double ref_lat, double ref_lon)
{
    double dlat = (lat - ref_lat) * DEG_TO_RAD;
    double dlon = (lon - ref_lon) * DEG_TO_RAD;
    double x = EARTH_RADIUS_M * dlon * std::cos(ref_lat * DEG_TO_RAD);
    double y = EARTH_RADIUS_M * dlat;
    return {x, y};
}

struct MissionPoint
{
    double x;
    double y;
    double tolerance;
};

std::vector<MissionPoint> parse_plan_file(const std::string &filename, double tolerance = 1.0)
{
    std::ifstream file(filename);
    if (!file.is_open())
        throw std::runtime_error("Impossible d'ouvrir le fichier de mission");

    json j;
    file >> j;

    std::vector<MissionPoint> waypoints;
    auto mission_array = j["mission"]["items"];

    if (mission_array.empty())
        throw std::runtime_error("Mission vide");

    double ref_lat = mission_array[0]["params"][4].get<double>();
    double ref_lon = mission_array[0]["params"][5].get<double>();

    for (const auto &item : mission_array)
    {
        double lat = item["params"][4];
        double lon = item["params"][5];
        auto [x, y] = latlon_to_xy(lat, lon, ref_lat, ref_lon);
        waypoints.push_back({x, y, tolerance});
    }
    return waypoints;
}

class NavigationController : public rclcpp::Node
{
public:
    NavigationController() : Node("navigation_controller")
    {
        declare_parameter<std::string>("mission_file", "");
        std::string mission_file = get_parameter("mission_file").as_string();

        try
        {
            mission_points_ = parse_plan_file(mission_file);
            RCLCPP_INFO(get_logger(), "Mission chargée avec %zu points", mission_points_.size());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Erreur chargement mission: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&NavigationController::odom_callback, this, std::placeholders::_1));

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/navigation_cmd", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&NavigationController::control_loop, this));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        latest_odom_ = *msg;
    }

    void control_loop()
    {
        if (!latest_odom_ || mission_points_.empty())
            return;

        if (current_waypoint_ >= mission_points_.size())
        {
            RCLCPP_INFO(get_logger(), "Mission terminée");
            publish_cmd(0, 0);
            timer_->cancel();
            return;
        }

        const auto &pose = latest_odom_->pose.pose;
        double current_x = pose.position.x;
        double current_y = pose.position.y;

        tf2::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        double current_heading = yaw;

        const MissionPoint &target = mission_points_[current_waypoint_];
        double dx = target.x - current_x;
        double dy = target.y - current_y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < target.tolerance)
        {
            RCLCPP_INFO(get_logger(), "Waypoint %zu atteint (dist=%.2fm)", current_waypoint_, dist);
            current_waypoint_++;
            return;
        }

        double target_bearing = std::atan2(dy, dx);
        double angle_error = normalize_angle(target_bearing - current_heading);

        double linear_speed = std::min(dist * 0.5, 1.0);
        double angular_speed = angle_error * 1.0;

        publish_cmd(linear_speed, angular_speed);
    }

    void publish_cmd(double lin, double ang)
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = lin;
        msg.angular.z = ang;
        cmd_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "/navigation_cmd : %s", msg);
    }

    double normalize_angle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::optional<nav_msgs::msg::Odometry> latest_odom_;
    std::vector<MissionPoint> mission_points_;
    size_t current_waypoint_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
