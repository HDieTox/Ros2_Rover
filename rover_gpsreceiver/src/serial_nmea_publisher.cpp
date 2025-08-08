#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <libserial/SerialPort.h>
#include <vector>
#include <algorithm>

using namespace LibSerial;

class NmeaPublisherNode : public rclcpp::Node
{
public:
    NmeaPublisherNode() : Node("nmea_publisher_node")
    {
        nmea_pub_ = this->create_publisher<std_msgs::msg::String>("/nmea", 10);

        // Port série pour Ubuntu 24.04 sur CM5
        std::string port = "/dev/ttyAMA0";  // Essayer aussi "/dev/ttyAMA0" ou "/dev/ttyUSB0" si nécessaire

        try
        {
            serial_port_.Open(port);
            serial_port_.SetBaudRate(BaudRate::BAUD_115200);
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
            serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
            serial_port_.SetParity(Parity::PARITY_NONE);
            serial_port_.SetStopBits(StopBits::STOP_BITS_1);
            
            RCLCPP_INFO(get_logger(), "Port série %s ouvert avec succès", port.c_str());
        }
        catch (const OpenFailed &)
        {
            RCLCPP_FATAL(get_logger(), "Échec d'ouverture du port %s", port.c_str());
            rclcpp::shutdown();
        }
        catch (const std::exception& e)
        {
            RCLCPP_FATAL(get_logger(), "Erreur initialisation: %s", e.what());
            rclcpp::shutdown();
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // Fréquence de lecture
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
        if (!serial_port_.IsOpen()) {
            return;
        }

        try
        {
            // Lire toutes les données disponibles sans timeout
            std::string data;
            while (serial_port_.IsDataAvailable()) {
                char c;
                serial_port_.ReadByte(c);  // Lecture non bloquante
                data += c;
            }

            if (!data.empty()) {
                // Ajouter les nouvelles données au buffer
                buffer_ += data;
                
                // Traiter toutes les lignes complètes
                size_t pos = 0;
                while ((pos = buffer_.find('\n')) != std::string::npos) {
                    if (pos == 0) {  // Ligne vide
                        buffer_.erase(0, 1);
                        continue;
                    }
                    
                    std::string line = buffer_.substr(0, pos);
                    buffer_.erase(0, pos + 1);
                    
                    // Nettoyer les retours chariot
                    line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
                    
                    if (!line.empty() && line[0] == '$') {
                        auto msg = std_msgs::msg::String();
                        msg.data = line;
                        nmea_pub_->publish(msg);
                        RCLCPP_INFO(this->get_logger(), "NMEA publié: %s", line.c_str());
                    }
                }
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Erreur lecture: %s", e.what());
            // Tentative de réouverture en cas d'erreur
            try {
                if (!serial_port_.IsOpen()) {
                    serial_port_.Open(port_);
                }
            } catch (...) {
                RCLCPP_ERROR(get_logger(), "Échec réouverture du port série");
            }
        }
    }

    SerialPort serial_port_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nmea_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string buffer_;
    std::string port_ = "/dev/ttyAMA0";  // Stocker le port pour réouverture
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NmeaPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}