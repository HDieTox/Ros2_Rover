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
        nmea_pub_ = this->create_publisher<std_msgs::msg::String>("/nmea", 10);

        // Modification du port série pour Ubuntu 24.04
        std::string port = "/dev/ttyS0"; // Changé ttyAMA0 → ttyS0

        try
        {
            serial_port_.Open(port);
            serial_port_.SetBaudRate(BaudRate::BAUD_38400);
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
            serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
            serial_port_.SetParity(Parity::PARITY_NONE);
            serial_port_.SetStopBits(StopBits::STOP_BITS_1);

            // Ajout d'un délai pour éviter les timeouts immédiats
            serial_port_.SetTimeout(1000); // Timeout de 1000 ms

            RCLCPP_INFO(get_logger(), "Port série %s ouvert avec succès", port.c_str());
        }
        catch (const OpenFailed &)
        {
            RCLCPP_FATAL(get_logger(), "Échec d'ouverture du port %s", port.c_str());
            rclcpp::shutdown();
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(get_logger(), "Erreur initialisation: %s", e.what());
            rclcpp::shutdown();
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // Réduire la fréquence (100ms)
            std::bind(&NmeaPublisherNode::readSerialData, this));
    }

    ~NmeaPublisherNode()
    {
        if (serial_port_.IsOpen())
        {
            serial_port_.Close();
        }
    }

private:
    void readSerialData()
    {
        if (!serial_port_.IsOpen())
        {
            return;
        }

        try
        {
            std::string line;
            serial_port_.ReadLine(line, '\n', 1000); // Timeout augmenté

            if (!line.empty() && line[0] == '$')
            {
                auto msg = std_msgs::msg::String();
                msg.data = line;
                nmea_pub_->publish(msg);
                RCLCPP_DEBUG(this->get_logger(), "NMEA publié: %s", line.c_str());
            }
        }
        // Gestion spécifique du timeout
        catch (const ReadTimeout &)
        {
            // Timeout normal, pas d'erreur fatale
            RCLCPP_DEBUG(this->get_logger(), "Timeout lecture (aucune donnée)");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Erreur lecture: %s", e.what());
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