#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

class PPMConverter : public rclcpp::Node {
public:
    PPMConverter() : Node("ppm_converter") {
        // Paramètres configurables
        declare_parameter("channel_linear", 1);
        declare_parameter("channel_angular", 2);
        declare_parameter("deadzone", 50);
        declare_parameter("ppm_min", 1000);
        declare_parameter("ppm_max", 2000);

        // Subscriber configurable
        ppm_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
            get_parameter("/ppm_manual_raw").as_string(), 10,
            [this](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
                processPPM(msg);
            });
        
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            get_parameter("/cmd_vel_manual").as_string(), 10);
    }

private:
    void processPPM(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        auto cmd_vel = geometry_msgs::msg::Twist();
        
        int ch_lin = get_parameter("channel_linear").as_int();
        int ch_ang = get_parameter("channel_angular").as_int();
        int deadzone = get_parameter("deadzone").as_int();

        cmd_vel.linear.x = normalize(msg->data[ch_lin], 
            get_parameter("ppm_min").as_int(),
            get_parameter("ppm_max").as_int(),
            deadzone);
            
        cmd_vel.angular.z = normalize(msg->data[ch_ang],
            get_parameter("ppm_min").as_int(),
            get_parameter("ppm_max").as_int(),
            deadzone);

        cmd_vel_pub_->publish(cmd_vel);
    }

    double normalize(int value, int min, int max, int deadzone) {
        double middle = (min + max) / 2.0;
        double range = (max - min) / 2.0;
        double val = (value - middle) / range; // [-1.0, +1.0] pour 1000-2000µs

        return (fabs(val) < deadzone/100.0) ? 0.0 : val;
    }

    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr ppm_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PPMConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}