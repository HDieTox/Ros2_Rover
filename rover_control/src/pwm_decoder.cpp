#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include <gpiod.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <memory>

class PWMDecoder : public rclcpp::Node {
public:
    PWMDecoder() : Node("pwm_decoder"), running_(true) {
        chip_ = gpiod_chip_open_by_name("gpiochip0");
        if (!chip_) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open gpiochip0");
            rclcpp::shutdown();
            return;
        }

        if (!setup_gpio_line(pwm_pin1_, &line1_) || !setup_gpio_line(pwm_pin2_, &line2_)) {
            cleanup();
            rclcpp::shutdown();
            return;
        }

        pwm_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("/pwm_manual_raw", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), [this]() { publishPWM(); });

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
    bool setup_gpio_line(unsigned int pin, struct gpiod_line **line) {
        *line = gpiod_chip_get_line(chip_, pin);
        if (!*line) {
            RCLCPP_FATAL(this->get_logger(), "Failed to get line %u", pin);
            return false;
        }

        // Demande la ligne en mode événement sur les deux fronts
        if (gpiod_line_request_both_edges_events(*line, "pwm_decoder") < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to request events for line %u", pin);
            return false;
        }

        return true;
    }

    void cleanup() {
        if (line1_) {
            gpiod_line_release(line1_);
            line1_ = nullptr;
        }
        if (line2_) {
            gpiod_line_release(line2_);
            line2_ = nullptr;
        }
        if (chip_) {
            gpiod_chip_close(chip_);
            chip_ = nullptr;
        }
    }

    void eventLoop() {
        struct gpiod_line_event event;
        auto last_rise1 = std::chrono::steady_clock::time_point{};
        auto last_rise2 = std::chrono::steady_clock::time_point{};

        struct timespec timeout = {0, 10000000}; // 10 ms

        while (rclcpp::ok() && running_) {
            check_line_event(line1_, 1, event, last_rise1, timeout);
            check_line_event(line2_, 2, event, last_rise2, timeout);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void check_line_event(struct gpiod_line *line, int channel,
                          struct gpiod_line_event &event,
                          std::chrono::steady_clock::time_point &last_rise,
                          const struct timespec &timeout) {
        int ret = gpiod_line_event_wait(line, &timeout);
        if (ret <= 0)
            return;

        ret = gpiod_line_event_read(line, &event);
        if (ret < 0)
            return;

        if (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
            last_rise = std::chrono::steady_clock::now();
        } else if (event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) {
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - last_rise);
            pwm_values_[channel - 1] = static_cast<uint16_t>(duration.count());
        }
    }

    void publishPWM() {
        auto msg = std_msgs::msg::Int16MultiArray();
        msg.data = {static_cast<int16_t>(pwm_values_[0]), static_cast<int16_t>(pwm_values_[1])};
        pwm_pub_->publish(msg);
    }

    // Configuration GPIO
    const unsigned int pwm_pin1_ = 17;
    const unsigned int pwm_pin2_ = 18;

    // Libgpiod
    struct gpiod_chip *chip_ = nullptr;
    struct gpiod_line *line1_ = nullptr;
    struct gpiod_line *line2_ = nullptr;

    // Contrôle
    std::atomic<bool> running_;
    std::thread event_thread_;
    uint16_t pwm_values_[2] = {0};

    // ROS2
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pwm_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PWMDecoder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

