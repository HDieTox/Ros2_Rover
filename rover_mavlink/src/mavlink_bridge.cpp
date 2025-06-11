#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mission/mission.h>

class MavlinkBridge : public rclcpp::Node {
public:
  MavlinkBridge() : Node("mavlink_bridge") {
    // Connexion au système MAVLink
    mavsdk::Mavsdk mavsdk;
    mavsdk.add_any_connection("udp://0.0.0.0:14550");
    
    auto system = get_system(mavsdk);
    auto mission = std::make_shared<mavsdk::Mission>(system);
    
    publisher_ = create_publisher<mavros_msgs::msg::WaypointList>("/mission_waypoints", 10);
    
    mission->subscribe_mission_progress(
      [this, mission](mavsdk::Mission::MissionProgress progress) {
        publishWaypoints(mission);
      });
  }

private:
  void publishWaypoints(std::shared_ptr<mavsdk::Mission> mission) {
    auto result = mission->download_mission();
    if (result.first != mavsdk::Mission::Result::Success) return;
    
    mavros_msgs::msg::WaypointList msg;
    for (const auto& item : result.second) {
      mavros_msgs::msg::Waypoint wp;
      wp.x_lat = item.latitude_deg;
      wp.y_long = item.longitude_deg;
      // ... autres paramètres ...
      msg.waypoints.push_back(wp);
    }
    publisher_->publish(msg);
  }

  rclcpp::Publisher<mavros_msgs::msg::WaypointList>::SharedPtr publisher_;
};