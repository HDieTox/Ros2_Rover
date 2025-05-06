#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <LibSerial/SerialPort.h>
#include <sstream>
#include <iomanip>
#include <cmath>

using namespace LibSerial;

class PPMConverter : public rclcpp::Node {
public:
    PPMConverter() : Node("ppm_converter") {
        declare_parameter("channel_linear", 0);
        declare_parameter("channel_angular", 1);
        declare_parameter("deadzone", 50);
        declare_parameter("ppm_min", 1000);
        declare_parameter("ppm_max", 2000);

        std::string port = "/dev/serial0";
        int baudrate = 115200;

        try {
            serial_port_.Open(port);
            serial_port_.SetBaudRate(BaudRate::BAUD_115200);
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
            serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
            serial_port_.SetParity(Parity::PARITY_NONE);
            serial_port_.SetStopBits(StopBits::STOP_BITS_1);
            RCLCPP_INFO(get_logger(), "Port série %s ouvert.", port.c_str());
        } catch (const OpenFailed&) {
            RCLCPP_ERROR(get_logger(), "Impossible d'ouvrir le port série %s", port.c_str());
        }

        ppm_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
            "/pwm_manual_raw", 10,
            [this](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
                processPPM(msg);
            });
    }

    ~PPMConverter() {
        if (serial_port_.IsOpen()) {
            serial_port_.Close();
        }
    }

private:
    void processPPM(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        int ch_lin = get_parameter("channel_linear").as_int();
        int ch_ang = get_parameter("channel_angular").as_int();
        int deadzone = get_parameter("deadzone").as_int();

        if (ch_lin >= static_cast<int>(msg->data.size()) || ch_ang >= static_cast<int>(msg->data.size())) {
            RCLCPP_WARN(get_logger(), "Channel index out of range");
            return;
        }

        auto cmd_vel = geometry_msgs::msg::Twist();

        cmd_vel.linear.x = normalize(msg->data[ch_lin], 
            get_parameter("ppm_min").as_int(),
            get_parameter("ppm_max").as_int(),
            deadzone);
        cmd_vel.angular.z = normalize(msg->data[ch_ang],
            get_parameter("ppm_min").as_int(),
            get_parameter("ppm_max").as_int(),
            deadzone);

        if (serial_port_.IsOpen()) {
            std::ostringstream oss;
            oss << "L:" << std::fixed << std::setprecision(3) << cmd_vel.linear.x
                << " A:" << std::fixed << std::setprecision(3) << cmd_vel.angular.z << "\n";
            serial_port_.Write(oss.str());
        }
        RCLCPP_INFO(get_logger(), "PPM Linear: %f, Angular: %f", 
            cmd_vel.linear.x, cmd_vel.angular.z);
    }

    double normalize(int value, int min, int max, int deadzone) {
        double middle = (min + max) / 2.0;
        double range = (max - min) / 2.0;
        double val = (value - middle) / range; // [-1.0, +1.0]
        return (fabs(val) < deadzone/100.0) ? 0.0 : val;
    }

    SerialPort serial_port_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr ppm_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PPMConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
