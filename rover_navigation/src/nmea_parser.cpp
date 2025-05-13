#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <nmea_msgs/msg/sentence.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nmea.h>
#include <nmea/gpgga.h>

class NMEAParser : public rclcpp::Node
{
public:
  NMEAParser() : Node("nmea_parser")
  {
    nmea_sub_ = create_subscription<std_msgs::msg::String>(
        "/nmea", 10,
        [this](const std_msgs::msg::String msg){
          nmea_callback(msg);
        });

    gps_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);

    RCLCPP_INFO(this->get_logger(), "NMEA Parser Node started");
  }

private:
  void nmea_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received NMEA length: %ld", msg.length());

    nmea_s *data = nmea_parse(msg, strlen(msg.c_str()), 0);

    if (data != NULL)
    {
      RCLCPP_INFO(this->get_logger(), "Type: %d", data->type);

      if (data->type == NMEA_GPGGA)
      {
        nmea_gpgga_s *gpgga = (nmea_gpgga_s *)data;

        auto fix = sensor_msgs::msg::NavSatFix();
        fix.header.stamp = get_clock()->now();
        fix.latitude = gpgga->latitude.degrees + (gpgga->latitude.minutes / 60.0);
        fix.longitude = gpgga->longitude.degrees + (gpgga->longitude.minutes / 60.0);
        fix.altitude = gpgga->altitude;
        RCLCPP_INFO(this->get_logger(), "Latitude=%f :: Longitude=%f :: Altitude=%f :: \n", fix.latitude, fix.longitude, fix.altitude);
        gps_pub_->publish(fix);
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "DATA ++ NULL :: Failed to parse NMEA sentence: %s", msg.c_str());
    }

    nmea_free(data);
  }

  rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr nmea_sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NMEAParser>());
  rclcpp::shutdown();
  return 0;
}