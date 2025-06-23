#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mission/mission.h>
#include <iterator>

class MavlinkBridge : public rclcpp::Node
{
public:
  MavlinkBridge() : Node("mavlink_bridge"), mavsdk_()
  {
    mavsdk_.add_any_connection("udp://0.0.0.0:14550");
    auto system = get_system(mavsdk_);
    mission_ = std::make_shared<mavsdk::Mission>(system);

    publisher_ = create_publisher<mavros_msgs::msg::WaypointList>("/mission_waypoints", 10);

    mission_->subscribe_mission_progress(
        [this, mission = mission_](mavsdk::Mission::MissionProgress progress)
        {
          publishWaypoints(mission);
        });
  }

private:
  void publishWaypoints(std::shared_ptr<mavsdk::Mission> mission)
  {
    auto result = mission->download_mission();
    if (result.first != mavsdk::Mission::Result::Success)
      return;

    mavros_msgs::msg::WaypointList msg;
    for (const auto &item : result.second)
    {
      mavros_msgs::msg::Waypoint wp;
      wp.x_lat = item.latitude_deg;
      wp.y_long = item.longitude_deg;
      // ... autres paramètres ...
      msg.waypoints.push_back(wp);
    }
    publisher_->publish(msg);
  }

  mavsdk::Mavsdk mavsdk_;
  std::shared_ptr<mavsdk::Mission> mission_;
  rclcpp::Publisher<mavros_msgs::msg::WaypointList>::SharedPtr publisher_;
};