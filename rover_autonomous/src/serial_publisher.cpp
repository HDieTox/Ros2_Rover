#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <libserial/SerialPort.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>

class SerialCommandPublisher : public rclcpp::Node
{
public:
    SerialCommandPublisher() : Node("serial_command_publisher")
    {
        // Initialize serial port
        try
        {
            serial_port_.Open("/dev/ttyAMA3");
            serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
            serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
            RCLCPP_INFO(get_logger(), "Port série ouvert avec succès");
        }
        catch (const LibSerial::OpenFailed &e)
        {
            RCLCPP_FATAL(get_logger(), "Impossible d'ouvrir le port série: %s", e.what());
            rclcpp::shutdown();
            return;
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(get_logger(), "Erreur série: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // Create subscription for command messages
        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/navigation_cmd", 10,
            std::bind(&SerialCommandPublisher::cmd_callback, this, std::placeholders::_1));

        // Create publisher for IMU data
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu_data", 10);

        // Start serial reading thread
        read_thread_ = std::thread(&SerialCommandPublisher::read_serial_thread, this);
    }

    ~SerialCommandPublisher()
    {
        // Signal thread to stop
        running_ = false;

        if (read_thread_.joinable())
        {
            read_thread_.join();
        }

        if (serial_port_.IsOpen())
        {
            serial_port_.Close();
            RCLCPP_INFO(get_logger(), "Port série fermé");
        }
    }

private:
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(serial_mutex_);

        if (!serial_port_.IsOpen())
        {
            RCLCPP_WARN(get_logger(), "Port série fermé, commande non envoyée");
            return;
        }

        // Format: L:<linear> A:<angular> avec 3 décimales, suivi d'un \n
        std::ostringstream ss;
        ss << "L:" << std::fixed << std::setprecision(3) << msg->linear.x
           << " A:" << std::fixed << std::setprecision(3) << msg->angular.z << "\n";

        std::string command_str = ss.str();

        try
        {
            serial_port_.Write(command_str);
            RCLCPP_INFO(get_logger(), "Envoyé au port série: %s", command_str.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Erreur écriture série: %s", e.what());
        }
    }

    void read_serial_thread()
    {
        RCLCPP_INFO(get_logger(), "Thread de lecture série démarré");

        std::string buffer;
        std::string read_data;
        constexpr size_t MAX_READ_BYTES = 256;

        while (rclcpp::ok() && running_)
        {
            try
            {
                size_t bytes_read = 0;
                {
                    std::lock_guard<std::mutex> lock(serial_mutex_);
                    if (!serial_port_.IsOpen() || !serial_port_.IsDataAvailable())
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        continue;
                    }

                    // La méthode Read remplit la string passée en argument
                    // avec les données lues et retourne le nombre d'octets lus
                    bytes_read = serial_port_.Read(read_data, 0, MAX_READ_BYTES);
                }

                if (bytes_read > 0)
                {
                    // read_data contient les données lues, mais attention à l'accumulation:
                    // on peut ajouter au buffer général
                    buffer.append(read_data);
                    // vider read_data pour prochaine lecture
                    read_data.clear();

                    // Le reste de ton traitement inchangé
                    size_t pos;
                    while ((pos = buffer.find('\n')) != std::string::npos)
                    {
                        if (pos > 0)
                        {
                            std::string line = buffer.substr(0, pos);
                            buffer.erase(0, pos + 1);

                            if (line.find("IMU,") == 0)
                            {
                                process_imu_line(line);
                            }
                        }
                        else
                        {
                            buffer.erase(0, 1);
                        }
                    }
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(get_logger(), "Erreur lecture série: %s", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }

    void process_imu_line(const std::string &line)
    {
        // Format: "IMU,timestamp,ax,ay,az,gx,gy,gz"
        std::vector<std::string> parts;
        std::istringstream iss(line);
        std::string part;

        while (std::getline(iss, part, ','))
        {
            parts.push_back(part);
        }

        // Validate data format
        if (parts.size() != 8 || parts[0] != "IMU")
        {
            RCLCPP_WARN(get_logger(), "Format IMU invalide: %s", line.c_str());
            return;
        }

        try
        {
            // Parse IMU data
            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = this->now();
            imu_msg.header.frame_id = "imu_link";

            // Convert acceleration (g to m/s²)
            imu_msg.linear_acceleration.x = std::stof(parts[2]) * 9.80665;
            imu_msg.linear_acceleration.y = std::stof(parts[3]) * 9.80665;
            imu_msg.linear_acceleration.z = std::stof(parts[4]) * 9.80665;

            // Convert gyro (deg/s to rad/s)
            constexpr float DEG_TO_RAD = M_PI / 180.0f;
            imu_msg.angular_velocity.x = std::stof(parts[5]) * DEG_TO_RAD;
            imu_msg.angular_velocity.y = std::stof(parts[6]) * DEG_TO_RAD;
            imu_msg.angular_velocity.z = std::stof(parts[7]) * DEG_TO_RAD;

            // Publish IMU message
            imu_pub_->publish(imu_msg);
            RCLCPP_DEBUG(get_logger(), "IMU publié: accel=(%.3f, %.3f, %.3f) gyro=(%.3f, %.3f, %.3f)",
                         imu_msg.linear_acceleration.x,
                         imu_msg.linear_acceleration.y,
                         imu_msg.linear_acceleration.z,
                         imu_msg.angular_velocity.x,
                         imu_msg.angular_velocity.y,
                         imu_msg.angular_velocity.z);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Erreur parsing IMU: %s - %s", e.what(), line.c_str());
        }
    }

    LibSerial::SerialPort serial_port_;
    std::mutex serial_mutex_;
    std::thread read_thread_;
    bool running_ = true;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialCommandPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}