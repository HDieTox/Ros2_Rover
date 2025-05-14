#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "rover_navigation/minmea.h"

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
    const std::string &sentence = msg->data;

    if (sentence.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Received empty NMEA sentence");
      return;
    }

    // Minmea attend une chaîne C terminée par '\0'
    const char *nmea_str = sentence.c_str();

    // Parser la phrase NMEA (vérifie checksum automatiquement)
    if (minmea_check(nmea_str,false))
    {
      // Identifier le type de phrase
      enum minmea_sentence_id id = minmea_sentence_id(nmea_str, false);

      if (id == MINMEA_SENTENCE_GGA)
      {
        struct minmea_sentence_gga frame;
        if (minmea_parse_gga(&frame, nmea_str))
        {
          // Convertir latitude et longitude en degrés décimaux
          double latitude = minmea_tocoord(&frame.latitude);
          double longitude = minmea_tocoord(&frame.longitude);

          // Créer et remplir le message NavSatFix
          auto fix = sensor_msgs::msg::NavSatFix();
          fix.header.stamp = this->now();
          fix.header.frame_id = "gps";

          fix.latitude = latitude;
          fix.longitude = longitude;
          fix.altitude = frame.altitude;

          // Statut du fix GPS
          fix.status.status = (frame.fix_quality > 0) ? sensor_msgs::msg::NavSatStatus::STATUS_FIX : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
          fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

          RCLCPP_INFO(this->get_logger(), "GPS Fix: lat=%.6f, lon=%.6f, alt=%.2f (quality=%d)",
                      fix.latitude, fix.longitude, fix.altitude, frame.fix_quality);

          gps_pub_->publish(fix);
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "Failed to parse GGA sentence");
        }
      }
      else
      {
        RCLCPP_DEBUG(this->get_logger(), "Received NMEA sentence type %d not handled", id);
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Invalid NMEA sentence (checksum failed): %s", sentence.c_str());
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
