#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <libserial/SerialPort.h>
#include <iostream>
#include <iomanip>
#include <sstream>

class SerialCommandPublisher : public rclcpp::Node
{
public:
    SerialCommandPublisher() : Node("serial_command_publisher")
    {
        // Remplace "/dev/ttyUSB0" par ton port série réel
        try {
            serial_port_.Open("/dev/ttyUSB0");
            serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
            serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
            RCLCPP_INFO(get_logger(), "Port série ouvert avec succès");
        } catch (const LibSerial::OpenFailed& e) {
            RCLCPP_FATAL(get_logger(), "Impossible d'ouvrir le port série: %s", e.what());
            rclcpp::shutdown();
            return;
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "Erreur série: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/navigation_cmd", 10,
            std::bind(&SerialCommandPublisher::cmd_callback, this, std::placeholders::_1));
    }

    ~SerialCommandPublisher()
    {
        if (serial_port_.IsOpen()) {
            serial_port_.Close();
            RCLCPP_INFO(get_logger(), "Port série fermé");
        }
    }

private:
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!serial_port_.IsOpen()) {
            RCLCPP_WARN(get_logger(), "Port série fermé, commande non envoyée");
            return;
        }

        // Format: L:<linear> A:<angular> avec 3 décimales, suivi d’un \n
        std::ostringstream ss;
        ss << "L:" << std::fixed << std::setprecision(3) << msg->linear.x
           << " A:" << std::fixed << std::setprecision(3) << msg->angular.z << "\n";

        std::string command_str = ss.str();

        try {
            serial_port_.Write(command_str);
            RCLCPP_INFO(get_logger(), "Envoyé au port série: %s", command_str.c_str());
        } catch(const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Erreur écriture série: %s", e.what());
        }
    }

    LibSerial::SerialPort serial_port_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialCommandPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
