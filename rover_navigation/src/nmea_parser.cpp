#include "rclcpp/rclcpp.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "libnmea/nmea.h"

class NMEAParser : public rclcpp::Node
{
public:
    NMEAParser() : Node("nmea_parser")
    {
        subscription_ = create_subscription<nmea_msgs::msg::Sentence>(
            "/nmea", 10, std::bind(&NMEAParser::nmea_callback, this, std::placeholders::_1));

        publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
    }

private:
    void nmea_callback(const nmea_msgs::msg::Sentence::SharedPtr msg)
    {
        nmea_s data;
        if (nmea_parse(&data, msg->sentence.c_str(), msg->sentence.size()) == NMEA_OK)
        {
            if (data.type == NMEA_GPGGA)
            {
                auto fix = sensor_msgs::msg::NavSatFix();
                fix.header.stamp = get_clock()->now();
                fix.latitude = data.gpgga.latitude;
                fix.longitude = data.gpgga.longitude;
                fix.altitude = data.gpgga.altitude;
                publisher_->publish(fix);
            }
        }
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