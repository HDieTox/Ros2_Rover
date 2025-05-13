#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nmea.h>
#include <nmea/gpgga.h>

class NMEAParser : public rclcpp::Node
{
public:
  NMEAParser() : Node("nmea_parser")
  {
    // Subscriber sur std_msgs::msg::String (à adapter si besoin)
    nmea_sub_ = create_subscription<std_msgs::msg::String>(
        "/nmea", 10,
        std::bind(&NMEAParser::nmea_callback, this, std::placeholders::_1));

    gps_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);

    RCLCPP_INFO(this->get_logger(), "NMEA Parser Node started");
  }

private:
  void nmea_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg.compare(3, 5, "GNGGA") == 0)
    {
      msg.replace(3, 5, "GPGGA");
    }

    const std::string &str = msg->data;
    if (str.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Received empty NMEA sentence");
      return;
    }


    // Préparer buffer avec terminaison nulle
    std::vector<char> buffer(str.begin(), str.end());
    buffer.push_back('\0');

    // Appeler nmea_parse avec la taille sans le '\0'
    nmea_s *data = nmea_parse(buffer.data(), str.size(), 0);

    if (data != nullptr)
    {
      RCLCPP_DEBUG(this->get_logger(), "Parsed NMEA sentence type: %d", data->type);

      if (data->type == NMEA_GPGGA)
      {
        nmea_gpgga_s *gpgga = reinterpret_cast<nmea_gpgga_s *>(data);

        auto fix = sensor_msgs::msg::NavSatFix();
        fix.header.stamp = this->now();
        fix.header.frame_id = "gps";

        // Calcul latitude
        fix.latitude = gpgga->latitude.degrees + (gpgga->latitude.minutes / 60.0);
        fix.longitude = gpgga->longitude.degrees + (gpgga->longitude.minutes / 60.0);

        RCLCPP_INFO(this->get_logger(), "GPS Fix: lat=%.6f, lon=%.6f, alt=%.2f",
                    fix.latitude, fix.longitude, fix.altitude);

        gps_pub_->publish(fix);
      }
      else
      {
        RCLCPP_DEBUG(this->get_logger(), "NMEA sentence type %d not handled", data->type);
      }

      nmea_free(data);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Failed to parse NMEA sentence: %s", str.c_str());
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nmea_sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NMEAParser>());
  rclcpp::shutdown();
  return 0;
}
