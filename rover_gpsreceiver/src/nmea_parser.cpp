// nmea_parser.cpp
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "rover_gpsreceiver/minmea.h"

class NMEAParser : public rclcpp::Node
{
public:
    NMEAParser() : Node("nmea_parser")
    {
        nmea_sub_ = create_subscription<std_msgs::msg::String>(
            "/nmea", 10,
            std::bind(&NMEAParser::nmea_callback, this, std::placeholders::_1));
        gps_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
        RCLCPP_INFO(get_logger(), "NMEAParser started");
    }

private:
    void nmea_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string &nmea = msg->data;
        if (nmea.empty() || !minmea_check(nmea.c_str(), false))
        {
            RCLCPP_WARN(get_logger(), "Invalid or empty NMEA sentence");
            return;
        }
        if (minmea_sentence_id(nmea.c_str(), false) == MINMEA_SENTENCE_GGA)
        {
            struct minmea_sentence_gga frame;
            if (minmea_parse_gga(&frame, nmea.c_str()))
            {
                if (frame.fix_quality == 0)
                {
                    RCLCPP_WARN(get_logger(), "No GPS fix");
                    return;
                }
                auto fix = sensor_msgs::msg::NavSatFix();
                fix.header.stamp = now();
                fix.header.frame_id = "gps";
                fix.latitude = minmea_tocoord(&frame.latitude);
                fix.longitude = minmea_tocoord(&frame.longitude);
                fix.altitude = minmea_tofloat(&frame.altitude);
                fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
                RCLCPP_INFO(this->get_logger(), "Publishing GPS fix: lat=%.6f, lon=%.6f, alt=%.2f",
                            fix.latitude, fix.longitude, fix.altitude);
                gps_pub_->publish(fix);
            }
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
