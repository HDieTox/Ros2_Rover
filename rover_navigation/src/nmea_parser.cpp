#include <rclcpp/rclcpp.hpp>
#include <nmea_msgs/msg/sentence.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nmea.h>
#include <nmea/gpgga.h>

class NMEAParser : public rclcpp::Node
{
public:
  NMEAParser() : Node("nmea_parser")
  {
    subscription_ = create_subscription<nmea_msgs::msg::Sentence>(
        "/nmea", 10,
        [this](const nmea_msgs::msg::Sentence::SharedPtr msg)
        {
          nmea_callback(msg);
        });

    ppm_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
        "/pwm_manual_raw", 10,
        [this](const std_msgs::msg::Int16MultiArray::SharedPtr msg)
        {
          processPPM(msg);
        });

    publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);

    RCLCPP_INFO(this->get_logger(), "NMEA Parser Node started");
  }

private:
  void nmea_callback(const nmea_msgs::msg::Sentence::SharedPtr msg)
  {
    std::string sentence_copy = msg->sentence;
    RCLCPP_INFO(this->get_logger(), "Received NMEA length: %ld", sentence_copy.length());

    nmea_s *data = nmea_parse(&sentence_copy[0], strlen(&sentence_copy[0]), 0);

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
        publisher_->publish(fix);
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "DATA ++ NULL :: Failed to parse NMEA sentence: %s", msg->sentence.c_str());
    }

    nmea_free(data);
  }

  rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NMEAParser>());
  rclcpp::shutdown();
  return 0;
}