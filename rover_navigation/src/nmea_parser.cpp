#include <rclcpp/rclcpp.hpp>
#include <nmea_msgs/msg/sentence.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nmea.h>
#include <nmea/gpgga.h>

class NMEAParser : public rclcpp::Node {
public:
  NMEAParser() : Node("nmea_parser") {
    subscription_ = create_subscription<nmea_msgs::msg::Sentence>(
      "/nmea", 10, std::bind(&NMEAParser::nmea_callback, this, std::placeholders::_1));
    
    publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
  }

private:
  void nmea_callback(const nmea_msgs::msg::Sentence::SharedPtr msg) {
    std::string sentence_copy = msg->sentence;
    nmea_s* data = nmea_parse(&sentence_copy[0], sentence_copy.size(), 0);

    if(data != NULL) {
      if(data->type == NMEA_GPGGA) {
        nmea_gpgga_s* gpgga = (nmea_gpgga_s*) data;

        auto fix = sensor_msgs::msg::NavSatFix();
        fix.header.stamp = get_clock()->now();
        fix.latitude = gpgga->latitude;
        fix.longitude = gpgga->longitude;
        fix.altitude = gpgga->altitude;
        publisher_->publish(fix);
      }
    }
  }

  rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NMEAParser>());
  rclcpp::shutdown();
  return 0;
}