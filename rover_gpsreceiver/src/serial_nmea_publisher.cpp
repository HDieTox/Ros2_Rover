#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <libserial/SerialPort.h>

using namespace LibSerial;

class NmeaPublisherNode : public rclcpp::Node
{
public:
    NmeaPublisherNode()
        : Node("nmea_publisher_node")
    {
        // Publisher sur le topic /nmea
        nmea_pub_ = this->create_publisher<std_msgs::msg::String>("/nmea", 10);

        std::string port = "/dev/ttyUSB0";

        try
        {
            serial_port_.Open(port);
            serial_port_.SetBaudRate(BaudRate::BAUD_38400);
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
            serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
            serial_port_.SetParity(Parity::PARITY_NONE);
            serial_port_.SetStopBits(StopBits::STOP_BITS_1);
            RCLCPP_INFO(get_logger(), "Port série %s ouvert.", port.c_str());
        }
        catch (const OpenFailed &)
        {
            RCLCPP_ERROR(get_logger(), "Impossible d'ouvrir le port série %s", port.c_str());
        }

        // Timer pour lire périodiquement les données série
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&NmeaPublisherNode::readSerialData, this));
    }

    ~NmeaPublisherNode()
    {
        if (serial_port_.IsOpen()) {
            serial_port_.Close();
        }
    }

private:
    void readSerialData()
    {
        std::string line;
        serial_port_.ReadLine(line,'\n',500); // lit une ligne dans 'line'

        // Vérifier si la ligne est vide ou invalide
        if (line.empty() || line[0] != '$')
        {
            RCLCPP_WARN(this->get_logger(), "Invalid NMEA line: %s", line.c_str());
        }
        else
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
