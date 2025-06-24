#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <cstring>

class MavlinkBridge : public rclcpp::Node {
public:
    MavlinkBridge() : Node("mavlink_bridge") {
        // Paramètres configurables
        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        this->declare_parameter("baud_rate", 57600);

        // Récupération des paramètres
        serial_port_ = this->get_parameter("serial_port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();

        RCLCPP_INFO(this->get_logger(), "Configuration: Port série=%s, Baud=%d", 
                   serial_port_.c_str(), baud_rate_);

        // Initialisation série
        if (!setup_serial()) {
            RCLCPP_FATAL(this->get_logger(), "Échec configuration série");
            rclcpp::shutdown();
            return;
        }

        // Démarrer le thread de lecture série
        running_.store(true);
        serial_read_thread_ = std::thread(&MavlinkBridge::serial_read_loop, this);

        RCLCPP_INFO(this->get_logger(), "Module radio actif sur [%s]", serial_port_.c_str());
    }

    ~MavlinkBridge() {
        RCLCPP_INFO(this->get_logger(), "Arrêt du module radio...");
        running_.store(false);
        if (serial_read_thread_.joinable()) serial_read_thread_.join();
        if (serial_fd_ != -1) close(serial_fd_);
        RCLCPP_INFO(this->get_logger(), "Ressources libérées");
    }

    // Fonction publique pour envoyer des données via le port série
    void send_serial_data(const uint8_t* data, size_t length) {
        if (serial_fd_ != -1) {
            write(serial_fd_, data, length);
            RCLCPP_DEBUG(this->get_logger(), "Envoi série: %ld octets", length);
        }
    }

private:
    bool setup_serial() {
        RCLCPP_INFO(this->get_logger(), "Ouverture port série: %s", serial_port_.c_str());
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Erreur ouverture port: %s", strerror(errno));
            return false;
        }

        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Erreur tcgetattr: %s", strerror(errno));
            return false;
        }

        // Configuration série
        cfsetospeed(&tty, baud_rate_);
        cfsetispeed(&tty, baud_rate_);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag = 0;
        tty.c_lflag = 0;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Erreur tcsetattr: %s", strerror(errno));
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Port série configuré: %d bauds", baud_rate_);
        return true;
    }

    void serial_read_loop() {
        RCLCPP_INFO(this->get_logger(), "Démarrage lecture série");
        uint8_t buffer[2048];
        bool first_packet = true;

        while (rclcpp::ok() && running_.load()) {
            ssize_t n = read(serial_fd_, buffer, sizeof(buffer));
            if (n > 0) {
                if (first_packet) {
                    RCLCPP_INFO(this->get_logger(), "Première trame radio reçue: %ld octets", n);
                    first_packet = false;
                }
                
                // Traitement des données MAVLink reçues
                process_mavlink_data(buffer, n);
                
            } else if (n < 0) {
                RCLCPP_ERROR(this->get_logger(), "Erreur lecture série: %s", strerror(errno));
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        RCLCPP_INFO(this->get_logger(), "Thread lecture série terminé");
    }

    void process_mavlink_data(const uint8_t* data, size_t length) {
        // ICI: Insérer le traitement des messages MAVLink
        // Exemple: décodage des messages, publication ROS, etc.
        RCLCPP_DEBUG(this->get_logger(), "Trame radio reçue: %ld octets", length);
        
        // Pour debug: affichage hexadécimal des premiers octets
        std::string hex_dump;
        for (size_t i = 0; i < std::min<size_t>(16, length); ++i) {
            char byte_str[4];
            snprintf(byte_str, sizeof(byte_str), "%02X ", data[i]);
            hex_dump += byte_str;
        }
        RCLCPP_DEBUG(this->get_logger(), "Début trame: %s%s", 
                    hex_dump.c_str(), (length > 16) ? "..." : "");
    }

    // Variables membres
    std::string serial_port_;
    int baud_rate_;
    
    int serial_fd_ = -1;
    std::thread serial_read_thread_;
    std::atomic<bool> running_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MavlinkBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
