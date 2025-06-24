#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <thread>
#include <atomic>
#include <cstring>

class MavlinkBridge : public rclcpp::Node {
public:
    MavlinkBridge() : Node("mavlink_bridge") {
        // Paramètres configurables
        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        this->declare_parameter("baud_rate", 57600);
        this->declare_parameter("udp_ip", "10.51.11.159");
        this->declare_parameter("udp_port", 14550);

        // Récupération des paramètres
        serial_port_ = this->get_parameter("serial_port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        udp_ip_ = this->get_parameter("udp_ip").as_string();
        udp_port_ = this->get_parameter("udp_port").as_int();

        RCLCPP_INFO(this->get_logger(), "Configuration: Port série=%s, Baud=%d, UDP=%s:%d", 
                   serial_port_.c_str(), baud_rate_, udp_ip_.c_str(), udp_port_);

        // Initialisation série
        if (!setup_serial()) {
            RCLCPP_FATAL(this->get_logger(), "Échec configuration série");
            rclcpp::shutdown();
            return;
        }

        // Initialisation UDP
        if (!setup_udp()) {
            RCLCPP_FATAL(this->get_logger(), "Échec configuration UDP");
            rclcpp::shutdown();
            return;
        }

        // Démarrer les threads
        running_.store(true);
        serial_to_udp_thread_ = std::thread(&MavlinkBridge::serial_to_udp, this);
        udp_to_serial_thread_ = std::thread(&MavlinkBridge::udp_to_serial, this);

        RCLCPP_INFO(this->get_logger(), "Pont MAVLink actif [%s] <=> [%s:%d]", 
                   serial_port_.c_str(), udp_ip_.c_str(), udp_port_);
    }

    ~MavlinkBridge() {
        RCLCPP_INFO(this->get_logger(), "Arrêt du pont MAVLink...");
        running_.store(false);
        if (serial_to_udp_thread_.joinable()) serial_to_udp_thread_.join();
        if (udp_to_serial_thread_.joinable()) udp_to_serial_thread_.join();
        if (serial_fd_ != -1) close(serial_fd_);
        if (udp_socket_ != -1) close(udp_socket_);
        RCLCPP_INFO(this->get_logger(), "Ressources libérées");
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

    bool setup_udp() {
        RCLCPP_INFO(this->get_logger(), "Configuration UDP: %s:%d", udp_ip_.c_str(), udp_port_);
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Erreur création socket: %s", strerror(errno));
            return false;
        }

        // Liaison du socket
        struct sockaddr_in local_addr;
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        local_addr.sin_port = htons(udp_port_);

        if (bind(udp_socket_, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Erreur bind: %s", strerror(errno));
            close(udp_socket_);
            return false;
        }

        // Configuration adresse distante
        memset(&udp_addr_, 0, sizeof(udp_addr_));
        udp_addr_.sin_family = AF_INET;
        udp_addr_.sin_port = htons(udp_port_);
        if (inet_pton(AF_INET, udp_ip_.c_str(), &udp_addr_.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Adresse IP invalide: %s", udp_ip_.c_str());
            close(udp_socket_);
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Socket UDP lié au port %d", udp_port_);
        return true;
    }

    void serial_to_udp() {
        RCLCPP_INFO(this->get_logger(), "Démarrage thread Série->UDP");
        uint8_t buffer[2048];
        bool first_packet = true;

        while (rclcpp::ok() && running_.load()) {
            ssize_t n = read(serial_fd_, buffer, sizeof(buffer));
            if (n > 0) {
                if (first_packet) {
                    RCLCPP_INFO(this->get_logger(), "Premier paquet série reçu: %ld octets", n);
                    first_packet = false;
                }
                
                sendto(udp_socket_, buffer, n, 0, 
                      (struct sockaddr*)&udp_addr_, sizeof(udp_addr_));
            } else if (n < 0) {
                RCLCPP_ERROR(this->get_logger(), "Erreur lecture série: %s", strerror(errno));
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        RCLCPP_INFO(this->get_logger(), "Thread Série->UDP terminé");
    }

    void udp_to_serial() {
        RCLCPP_INFO(this->get_logger(), "Démarrage thread UDP->Série");
        uint8_t buffer[2048];
        struct sockaddr_in src_addr;
        socklen_t addr_len = sizeof(src_addr);
        bool first_packet = true;

        while (rclcpp::ok() && running_.load()) {
            ssize_t n = recvfrom(udp_socket_, buffer, sizeof(buffer), 0,
                                (struct sockaddr*)&src_addr, &addr_len);
            if (n > 0) {
                if (first_packet) {
                    char ip_str[INET_ADDRSTRLEN];
                    inet_ntop(AF_INET, &src_addr.sin_addr, ip_str, INET_ADDRSTRLEN);
                    RCLCPP_INFO(this->get_logger(), "Premier paquet UDP reçu: %ld octets de %s:%d", 
                               n, ip_str, ntohs(src_addr.sin_port));
                    first_packet = false;
                }
                
                write(serial_fd_, buffer, n);
            } else if (n < 0) {
                RCLCPP_ERROR(this->get_logger(), "Erreur réception UDP: %s", strerror(errno));
            }
        }
        RCLCPP_INFO(this->get_logger(), "Thread UDP->Série terminé");
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
