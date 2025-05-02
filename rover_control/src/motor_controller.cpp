#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include <pigpiod_if2.h>

class MotorController : public rclcpp::Node {
public:
    MotorController() : Node("motor_controller") {
        // Paramètres
        declare_parameter("max_speed", 0.8);

        // Initialisation pigpio
        pi_ = pigpio_start(nullptr, nullptr);
        if (pi_< 0) {
            RCLCPP_FATAL(this->get_logger(), "Échec de la connexion à pigpiod !");
            rclcpp::shutdown();
        }
        // Configuration du GPIO en entrée
        set_mode(pi_, left_pwm_pin_, PI_OUTPUT);
        set_mode(pi_, right_pwm_pin_, PI_OUTPUT);

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

private:
    void updateMotors() {

        auto msg = selectControlSource();
        
        double max_speed = get_parameter("max_speed").as_double();
        double left = msg->linear.x - msg->angular.z;
        double right = msg->linear.x + msg->angular.z;

        setMotorSpeed(
            std::clamp(left * max_speed, -1.0, 1.0),
            std::clamp(right * max_speed, -1.0, 1.0)
        );
    }

    geometry_msgs::msg::Twist::SharedPtr selectControlSource() {
        return last_manual_cmd_;
    }

    void setMotorSpeed(double left, double right) {
        // Convertir [-1.0, 1.0] en PWM (1000-2000 µs pour le TReX Jr)
        int left_pwm = 1500 + (left * 500); // 1000µs (arrière) à 2000µs (avant)
        int right_pwm = 1500 + (right * 500);

        gpioServo(17, left_pwm);  // GPIO17 -> Moteur gauche
        gpioServo(18, right_pwm); // GPIO18 -> Moteur droit

        RCLCPP_INFO(get_logger(), "Motors: L=%.2f, R=%.2f", left, right);
    }

    // Configuration des broches GPIO (ajuster selon votre branchement)
    const unsigned left_pwm_pin_ = 22; // GPIO17 pour le canal 1 (Pin15)
    const unsigned right_pwm_pin_ = 23; // GPIO18 pour le canal 2 (Pin16)


    // Variables membres
    bool emergency_stop_ = false;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_cmd_sub_;
    geometry_msgs::msg::Twist::SharedPtr last_cmd_vel_;
    geometry_msgs::msg::Twist::SharedPtr last_manual_cmd_;
    rclcpp::Time last_manual_time_;
};
