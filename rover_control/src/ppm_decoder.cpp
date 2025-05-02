#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include <pigpiod_if2.h>

class PWMDecoder : public rclcpp::Node {
public:
    PWMDecoder() : Node("pwm_decoder") {
        // Initialisation pigpio
        pi_ = pigpio_start(nullptr, nullptr);
        if (pi_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Échec de la connexion à pigpiod !");
            rclcpp::shutdown();
        }

        // Configuration des GPIO en entrée pour les 2 canaux PWM
        set_mode(pi_, pwm_pin1_, PI_INPUT);
        set_mode(pi_, pwm_pin2_, PI_INPUT);

        // Callbacks pour mesurer les impulsions PWM
        callback1_ = callback_ex(pi_, pwm_pin1_, EITHER_EDGE, pwmIsrCallback, this);
        callback2_ = callback_ex(pi_, pwm_pin2_, EITHER_EDGE, pwmIsrCallback, this);

        if (callback1_ < 0 || callback2_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Échec de l'enregistrement des callbacks GPIO");
        }

        // Publisher pour les données PWM
        pwm_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("/pwm_manual_raw", 10);

        // Timer pour publier périodiquement
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),  // 50Hz
            [this]() { publishPWM(); });
    }

    ~PWMDecoder() {
        if (pi_ >= 0) {
            if (callback1_ >= 0) callback_cancel(callback1_);
            if (callback2_ >= 0) callback_cancel(callback2_);
            pigpio_stop(pi_);
        }
    }

private:
    static void pwmIsrCallback(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user) {
        auto self = static_cast<PWMDecoder*>(user);
        
        if (gpio == self->pwm_pin1_) {
            if (level == 1) {
                // Front montant - début de l'impulsion
                self->last_rise1_ = tick;
            } else {
                // Front descendant - fin de l'impulsion
                if (self->last_rise1_ != 0) {
                    uint32_t pulse_width = tick - self->last_rise1_;
                    // Convertir en microsecondes et stocker
                    self->pwm_values_[0] = pulse_width;
                }
            }
        } 
        else if (gpio == self->pwm_pin2_) {
            if (level == 1) {
                // Front montant - début de l'impulsion
                self->last_rise2_ = tick;
            } else {
                // Front descendant - fin de l'impulsion
                if (self->last_rise2_ != 0) {
                    uint32_t pulse_width = tick - self->last_rise2_;
                    // Convertir en microsecondes et stocker
                    self->pwm_values_[1] = pulse_width;
                }
            }
        }
    }

    void publishPWM() {
        auto msg = std_msgs::msg::Int16MultiArray();
        msg.data.push_back(pwm_values_[0]); // Canal 1
        msg.data.push_back(pwm_values_[1]); // Canal 2
        pwm_pub_->publish(msg);
    }

    // Configuration des broches GPIO (ajuster selon votre branchement)
    const unsigned pwm_pin1_ = 17; // GPIO17 pour le canal 1 (Pin11)
    const unsigned pwm_pin2_ = 18; // GPIO18 pour le canal 2 (Pin12)

    int pi_;
    int callback1_;
    int callback2_;
    uint32_t last_rise1_ = 0;
    uint32_t last_rise2_ = 0;
    uint16_t pwm_values_[2] = {0};
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pwm_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PWMDecoder>());
    rclcpp::shutdown();
    return 0;
}