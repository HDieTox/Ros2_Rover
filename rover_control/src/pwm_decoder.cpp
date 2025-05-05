#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include <gpiod.hpp>
#include <thread>
#include <chrono>

class PWMDecoder : public rclcpp::Node {
public:
    PWMDecoder() : Node("pwm_decoder") {
        try {
            // Ouverture du chip GPIO avec la nouvelle API
            chip_ = gpiod::chip("/dev/gpiochip0", gpiod::chip::OPEN_LOOKUP);
            
            // Obtention des lignes GPIO
            line1_ = chip_.get_line(pwm_pin1_);
            line2_ = chip_.get_line(pwm_pin2_);

            // Configuration des lignes en entrée avec détection des deux fronts
            gpiod::line_request config;
            config.consumer = "pwm_decoder";
            config.request_type = gpiod::line_request::EVENT_BOTH_EDGES;
            
            line1_.request(config);
            line2_.request(config);

            // Publisher ROS2
            pwm_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("/pwm_manual_raw", 10);

            // Timer pour publier périodiquement
            timer_ = this->create_wall_timer(std::chrono::milliseconds(20), [this]() { publishPWM(); });

            // Thread pour surveiller les événements GPIO
            event_thread_ = std::thread([this]() { eventLoop(); });

        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Erreur GPIO: %s", e.what());
            rclcpp::shutdown();
        }
    }

    ~PWMDecoder() {
        running_ = false;
        if (event_thread_.joinable()) {
            event_thread_.join();
        }
    }

private:
    void eventLoop() {
        auto last_rise1 = std::chrono::steady_clock::time_point{};
        auto last_rise2 = std::chrono::steady_clock::time_point{};

        while (rclcpp::ok() && running_) {
            try {
                // Vérifier les événements sur les deux lignes avec timeout
                if (line1_.event_wait(std::chrono::milliseconds(10))) {
                    auto event = line1_.event_read();
                    handleEvent(event, 1, last_rise1);
                }

                if (line2_.event_wait(std::chrono::milliseconds(10))) {
                    auto event = line2_.event_read();
                    handleEvent(event, 2, last_rise2);
                }

                // Petite pause pour éviter une boucle trop serrée
                std::this_thread::sleep_for(std::chrono::milliseconds(1));

            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Erreur dans la boucle d'événements: %s", e.what());
            }
        }
    }

    void handleEvent(const gpiod::line_event& event, int channel,
                    std::chrono::steady_clock::time_point& last_rise) {
        if (event.event_type == gpiod::line_event::RISING_EDGE) {
            last_rise = std::chrono::steady_clock::now();
        } else if (event.event_type == gpiod::line_event::FALLING_EDGE) {
            if (last_rise.time_since_epoch().count() != 0) {
                auto now = std::chrono::steady_clock::now();
                auto pulse_width = std::chrono::duration_cast<std::chrono::microseconds>(now - last_rise).count();
                if (channel == 1) {
                    pwm_values_[0] = static_cast<uint16_t>(pulse_width);
                } else {
                    pwm_values_[1] = static_cast<uint16_t>(pulse_width);
                }
            }
        }
    }

    void publishPWM() {
        auto msg = std_msgs::msg::Int16MultiArray();
        msg.data.push_back(pwm_values_[0]);
        msg.data.push_back(pwm_values_[1]);
        pwm_pub_->publish(msg);
    }

    const unsigned pwm_pin1_ = 17; // GPIO17
    const unsigned pwm_pin2_ = 18; // GPIO18

    gpiod::chip chip_;
    gpiod::line line1_;
    gpiod::line line2_;

    std::atomic<bool> running_ = true;
    std::thread event_thread_;

    uint16_t pwm_values_[2] = {0};
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pwm_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};