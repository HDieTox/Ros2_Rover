#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialPort.h>

class NmeaPublisherNode : public rclcpp::Node
{
public:
    NmeaPublisherNode()
        : Node("nmea_publisher_node")
    {
        // Publisher sur le topic /nmea
        nmea_pub_ = this->create_publisher<std_msgs::msg::String>("/nmea", 20);

        // Ouvre le port série UART du CM5 (exemple : "/dev/ttyS1", à adapter selon ta config)
        // Baudrate typique pour ZED-F9P UART : 115200 ou 460800 bps
        if (!serial_port_.Open("/dev/ttyAMA0", 38400))
        {
            RCLCPP_ERROR(this->get_logger(), "Impossible d'ouvrir le port série /dev/ttyAMA0");
            rclcpp::shutdown();
            return;
        }

        // Timer pour lire périodiquement les données série
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&NmeaPublisherNode::readSerialData, this));
    }

    ~NmeaPublisherNode()
    {
        serial_port_.Close();
    }

private:
    void readSerialData()
    {
        std::string line;
        // Lire une ligne complète NMEA (terminée par \n)
        while (serial_port_.ReadLine(line))
        {
            // Publier la ligne NMEA reçue sur /nmea
            auto msg = std_msgs::msg::String();
            msg.data = line;
            nmea_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published NMEA: %s", line.c_str());
        }
    }

    SerialPort serial_port_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nmea_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NmeaPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
