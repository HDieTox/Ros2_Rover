#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include <gpiod.h>
#include <thread>
#include <chrono>
#include <memory>

class PWMDecoder : public rclcpp::Node {
public:
    PWMDecoder() : Node("pwm_decoder") {
        // Configuration des broches GPIO
        const char* chipname = "gpiochip0";
        struct gpiod_chip* chip;
        struct gpiod_line* line1;
        struct gpiod_line* line2;

        // Ouvrir le chip GPIO
        chip = gpiod_chip_open_by_name(chipname);
        if (!chip) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open GPIO chip");
            rclcpp::shutdown();
            return;
        }

        // Obtenir les lignes GPIO
        line1 = gpiod_chip_get_line(chip, pwm_pin1_);
        line2 = gpiod_chip_get_line(chip, pwm_pin2_);
        if (!line1 || !line2) {
            RCLCPP_FATAL(this->get_logger(), "Failed to get GPIO lines");
            gpiod_chip_close(chip);
            rclcpp::shutdown();
            return;
        }

        // Configurer les lignes en entrée avec détection des deux fronts
        if (gpiod_line_request_both_edges_events(line1, "pwm_decoder") < 0 ||
            gpiod_line_request_both_edges_events(line2, "pwm_decoder") < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to configure GPIO events");
            gpiod_line_release(line1);
            gpiod_line_release(line2);
            gpiod_chip_close(chip);
            rclcpp::shutdown();
            return;
        }

        // Stocker les ressources
        chip_ = chip;
        line1_ = line1;
        line2_ = line2;

        // Publisher ROS2
        pwm_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("/pwm_manual_raw", 10);

        // Timer pour publier périodiquement
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), [this]() { publishPWM(); });

        // Thread pour surveiller les événements GPIO
        event_thread_ = std::thread([this]() { eventLoop(); });
    }

    ~PWMDecoder() {
        running_ = false;
        if (event_thread_.joinable()) {
            event_thread_.join();
        }
        cleanup();
    }

private:
    void cleanup() {
        if (line1_) gpiod_line_release(line1_);
        if (line2_) gpiod_line_release(line2_);
        if (chip_) gpiod_chip_close(chip_);
    }

    void eventLoop() {
        struct gpiod_line_event event;
        auto last_rise1 = std::chrono::steady_clock::time_point{};
        auto last_rise2 = std::chrono::steady_clock::time_point{};

        while (rclcpp::ok() && running_) {
            // Vérifier les événements sur line1
            if (gpiod_line_event_wait(line1_, &timeout_) > 0) {
                if (gpiod_line_event_read(line1_, &event) == 0) {
                    handleEvent(event, 1, last_rise1);
                }
            }

            // Vérifier les événements sur line2
            if (gpiod_line_event_wait(line2_, &timeout_) > 0) {
                if (gpiod_line_event_read(line2_, &event) == 0) {
                    handleEvent(event, 2, last_rise2);
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void handleEvent(const struct gpiod_line_event& event, int channel,
                    std::chrono::steady_clock::time_point& last_rise) {
        if (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
            last_rise = std::chrono::steady_clock::now();
        } else if (event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) {
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - last_