#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <thread>
#include <atomic>

class MavlinkBridge : public rclcpp::Node {
public:
    MavlinkBridge() : Node("mavlink_bridge") {
        // Paramètres configurables
        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        this->declare_parameter("baud_rate", 57600);
        this->declare_parameter("udp_ip", "10.51.11.159");  // IP du PC avec QGroundControl
        this->declare_parameter("udp_port", 14550);

        // Récupération des paramètres
        serial_port_ = this->get_parameter("serial_port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        udp_ip_ = this->get_parameter("udp_ip").as_string();
        udp_port_ = this->get_parameter("udp_port").as_int();

        // Initialisation du port série
        if (!setup_serial()) {
            RCLCPP_FATAL(this->get_logger(), "Échec de la configuration série");
            rclcpp::shutdown();
            return;
        }

        // Initialisation UDP
        if (!setup_udp()) {
            RCLCPP_FATAL(this->get_logger(), "Échec de la configuration UDP");
            rclcpp::shutdown();
            return;
        }

        // Démarrer les threads de transfert
        running_.store(true);
        serial_to_udp_thread_ = std::thread(&MavlinkBridge::serial_to_udp, this);
        udp_to_serial_thread_ = std::thread(&MavlinkBridge::udp_to_serial, this);

        RCLCPP_INFO(this->get_logger(), "Pont MAVLink actif [%s] <=> [%s:%d]", 
                   serial_port_.c_str(), udp_ip_.c_str(), udp_port_);
    }

    ~MavlinkBridge() {
        running_.store(false);
        if (serial_to_udp_thread_.joinable()) serial_to_udp_thread_.join();
        if (udp_to_serial_thread_.joinable()) udp_to_serial_thread_.join();
        if (serial_fd_ != -1) close(serial_fd_);
        if (udp_socket_ != -1) close(udp_socket_);
    }

private:
    bool setup_serial() {
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Erreur d'ouverture du port série: %s", strerror(errno));
            return false;
        }

        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Erreur tcgetattr: %s", strerror(errno));
            return false;
        }

        // Configuration du port série
        cfsetospeed(&tty, baud_rate_);
        cfsetispeed(&tty, baud_rate_);

        tty.c_cflag &= ~PARENB;   // Pas de parité
        tty.c_cflag &= ~CSTOPB;   // 1 bit stop
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;       // 8 bits
        tty.c_cflag &= ~CRTSCTS;  // Pas de contrôle de flux
        tty.c_cflag |= CREAD | CLOCAL;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Pas de contrôle de flux logiciel
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

        tty.c_oflag = 0;
        tty.c_lflag = 0;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Erreur tcsetattr: %s", strerror(errno));
            return false;
        }

        return true;
    }

    bool setup_udp() {
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Erreur création socket: %s", strerror(errno));
            return false;
        }

        memset(&udp_addr_, 0, sizeof(udp_addr_));
        udp_addr_.sin_family = AF_INET;
        udp_addr_.sin_port = htons(udp_port_);
        
        if (inet_pton(AF_INET, udp_ip_.c_str(), &udp_addr_.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Adresse IP invalide: %s", udp_ip_.c_str());
            return false;
        }

        return true;
    }

    void serial_to_udp() {
        uint8_t buffer[2048];
        while (rclcpp::ok() && running_.load()) {
            ssize_t n = read(serial_fd_, buffer, sizeof(buffer));
            if (n > 0) {
                sendto(udp_socket_, buffer, n, 0, 
                      (struct sockaddr*)&udp_addr_, sizeof(udp_addr_));
            } else if (n < 0) {
                RCLCPP_ERROR(this->get_logger(), "Erreur lecture série: %s", strerror(errno));
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    }

    void udp_to_serial() {
        uint8_t buffer[2048];
        socklen_t addr_len = sizeof(udp_addr_);
        
        while (rclcpp::ok() && running_.load()) {
            ssize_t n = recvfrom(udp_socket_, buffer, sizeof(buffer), 0,
                                (struct sockaddr*)&udp_addr_, &addr_len);
            if (n > 0) {
                write(serial_fd_, buffer, n);
            } else if (n < 0) {
                RCLCPP_ERROR(this->get_logger(), "Erreur réception UDP: %s", strerror(errno));
            }
        }
    }

    // Variables membres
    std::string serial_port_;
    int baud_rate_;
    std::string udp_ip_;
    int udp_port_;
    
    int serial_fd_ = -1;
    int udp_socket_ = -1;
    struct sockaddr_in udp_addr_;
    
    std::thread serial_to_udp_thread_;
    std::thread udp_to_serial_thread_;
    std::atomic<bool> running_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MavlinkBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}