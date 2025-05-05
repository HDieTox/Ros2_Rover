#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <libserial/SerialPort.h>
#include <stdexcept>

class MotorController : public rclcpp::Node {
public:
    MotorController() : Node("motor_controller") {
        // Déclaration du paramètre pour la vitesse maximale
        declare_parameter("max_speed", 0.8);

        // Initialisation du port série
        
        try {
            serial_port.Open("/dev/ttyAMA0");
            serial_port.SetBaudRate(LibSerial::BaudRate::BAUD_19200);
            serial_port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            serial_port.SetParity(LibSerial::Parity::PARITY_NONE);
            serial_port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            serial_port.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        } catch (const serial::IOException& e) {
            RCLCPP_FATAL(this->get_logger(), "Impossible d'ouvrir le port série: %s", e.what());
            rclcpp::shutdown();
        }
        if (serial_port.isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Port série ouvert");
        } else {
            RCLCPP_FATAL(this->get_logger(), "Impossible d'ouvrir le port série");
            rclcpp::shutdown();
        }

        // Subscribers
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                last_cmd_vel_ = msg;
            });

        manual_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_manual", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                last_manual_cmd_ = msg;
                last_manual_time_ = now();
            });

        // Timer
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() { updateMotors(); });
    }

    ~MotorController() {
        if (serial_.isOpen()) {
            serial_.close();
            RCLCPP_INFO(this->get_logger(), "Port série fermé");
        }
    }


private:
    void updateMotors() {
        auto msg = selectControlSource();

        if (!msg) {
            // Pas de commande, arrêter les moteurs
            setMotorSpeed(0.0, 0.0);
            return;
        }

        // Récupérer la vitesse maximale depuis les paramètres
        double max_speed = get_parameter("max_speed").as_double();

        // Calculer les vitesses pour chaque moteur
        double left = msg->linear.x - msg->angular.z;
        double right = msg->linear.x + msg->angular.z;

        // Limiter les vitesses
        setMotorSpeed(
            std::clamp(left * max_speed, -1.0, 1.0),
            std::clamp(right * max_speed, -1.0, 1.0));
    }

    geometry_msgs::msg::Twist::SharedPtr selectControlSource() {
        // Enovoie uniquement la commande manuelle 

        // AJOUTER LES COMMANDE AUTONOMES

        return last_manual_cmd_;
    }


    void setMotorSpeed(double left, double right) {
        // Convertir les vitesses [-1.0, 1.0] en commandes pour le TReX Jr (0-127 avant, 128-255 arrière)
        int left_speed = static_cast<int>(left * 63.0 + 63.0);  // Conversion et centrage
        int right_speed = static_cast<int>(right * 63.0 + 63.0);

        // Limiter les valeurs
        left_speed = std::clamp(left_speed, 0, 127);
        right_speed = std::clamp(right_speed, 0, 127);

        // Envoyer les commandes au TReX Jr
        sendTrexCommand(0xC0, left_speed);  // Moteur 1
        sendTrexCommand(0xC8, right_speed); // Moteur 2

        RCLCPP_INFO(get_logger(), "Motors: L=%d, R=%d", left_speed, right_speed);
    }

    void sendTrexCommand(uint8_t command, uint8_t value) {
        // Envoi de la commande au TReX Jr via le port série
        uint8_t data[2] = {command, value};
        serial_port.write(data, 2);
    }

    // Variables membres
    LibSerial::SerialPort serial_port;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_cmd_sub_;
    geometry_msgs::msg::Twist::SharedPtr last_cmd_vel_;
    geometry_msgs::msg::Twist::SharedPtr last_manual_cmd_;
    rclcpp::Time last_manual_time_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController>());
    rclcpp::shutdown();
    return 0;
}
