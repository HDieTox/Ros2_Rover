#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include <gpiod.h>
#include <thread>

class PWMDecoder : public rclcpp::Node {
public:
    PWMDecoder() : Node("pwm_decoder") {
        chip_ = gpiod_chip_open_by_name("gpiochip0");
        if (!chip_) {
            RCLCPP_FATAL(this->get_logger(), "Impossible d'ouvrir gpiochip0");
            rclcpp::shutdown();
            return;
        }

        line1_ = gpiod_chip_get_line(chip_, pwm_pin1_);
        line2_ = gpiod_chip_get_line(chip_, pwm_pin2_);

        if (!line1_ || !line2_) {
            RCLCPP_FATAL(this->get_logger(), "Impossible d'obtenir les lignes GPIO");
            rclcpp::shutdown();
            return;
        }

        // Configurer lignes en entrée avec détection front montant et descendant
        int ret1 = gpiod_line_request_both_edges_events(line1_, "pwm_decoder");
        int ret2 = gpiod_line_request_both_edges_events(line2_, "pwm_decoder");

        if (ret1 < 0 || ret2 < 0) {
            RCLCPP_FATAL(this->get_logger(), "Impossible de configurer les événements GPIO");
            rclcpp::shutdown();
            return;
        }

        // Publisher ROS2
        pwm_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("/pwm_manual_raw", 10);

        // Timer pour publier périodiquement
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), [this]() { publishPWM(); });

        // Thread pour surveiller les événements GPIO
        event_thread_ = std::thread([this]() { eventLoop(); });
    }

    ~PWMDecoder() {
        running_ = false;
        if (event_thread_.joinable()) event_thread_.join();

        if (line1_) gpiod_line_release(line1_);
        if (line2_) gpiod_line_release(line2_);
        if (chip_) gpiod_chip_close(chip_);
    }

private:
    void eventLoop() {
        struct gpiod_line_event event;
        auto last_rise1 = std::chrono::steady_clock::time_point{};
        auto last_rise2 = std::chrono::steady_clock::time_point{};

        while (rclcpp::ok() && running_) {
            // Timeout pour ne pas bloquer indéfiniment
            int ret1 = gpiod_line_event_wait(line1_, &timeout_);
            if (ret1 > 0) {
                if (gpiod_line_event_read(line1_, &event) == 0) {
                    handleEvent(event, 1, last_rise1);
                }
            }

            int ret2 = gpiod_line_event_wait(line2_, &timeout_);
            if (ret2 > 0) {
                if (gpiod_line_event_read(line2_, &event) == 0) {
                    handleEvent(event, 2, last_rise2);
                }
            }
            // Petite pause pour éviter une boucle trop serrée
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void handleEvent(const gpiod_line_event &event, int channel,
                     std::chrono::steady_clock::time_point &last_rise) {
        if (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
            last_rise = std::chrono::steady_clock::now();
        } else if (event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) {
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

    struct gpiod_chip *chip_ = nullptr;
    struct gpiod_line *line1_ = nullptr;
    struct gpiod_line *line2_ = nullptr;

    std::atomic<bool> running_ = true;
    std::thread event_thread_;
    struct timespec timeout_ = {0, 10000000}; // 10 ms

    uint16_t pwm_values_[2] = {0};
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pwm_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
