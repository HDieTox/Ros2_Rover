#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from gpiozero import Button
from time import perf_counter

class PWMDecoder(Node):
    def __init__(self):
        super().__init__('pwm_decoder')

        # Configuration des broches
        self.pwm_pins = [17, 18] # GPIO17 et GPIO18
        self.pwm_values = [0, 0]
        self.last_rise = [0, 0]

        self.pwm_history = [[], []]  # pour stocker les dernières mesures

        # Instanciation des entrées GPIO
        self.buttons = [Button(self.pwm_pins[0], pull_up=False, bounce_time=None),
                        Button(self.pwm_pins[1], pull_up=False, bounce_time=None)]

        # Attache les callbacks pour chaque broche
        self.buttons[0].when_pressed = lambda: self.on_rising(0)
        self.buttons[0].when_released = lambda: self.on_falling(0)
        self.buttons[1].when_pressed = lambda: self.on_rising(1)
        self.buttons[1].when_released = lambda: self.on_falling(1)

        # Publisher ROS2
        self.publisher_ = self.create_publisher(Int16MultiArray, "/pwm_manual_raw", 10)
        timer_period = 0.02  # 20 ms
        self.timer = self.create_timer(timer_period, self.publish_pwm)

        self.get_logger().info("PWMDecoder node started with gpiozero")

    def on_rising(self, channel):
        self.last_rise[channel] = perf_counter()


    def on_falling(self, channel):
        if self.last_rise[channel] != 0:
            pulse_width = (perf_counter() - self.last_rise[channel]) * 1_000_000
            if 500 <= pulse_width <= 2500:
                self.pwm_history[channel].append(pulse_width)
                if len(self.pwm_history[channel]) > 5:  # moyenne sur 5 mesures
                    self.pwm_history[channel].pop(0)
                self.pwm_values[channel] = int(sum(self.pwm_history[channel]) / len(self.pwm_history[channel]))


    def publish_pwm(self):
        msg = Int16MultiArray()
        msg.data = [int(self.pwm_values[0]), int(self.pwm_values[1])]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published PWM values: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = PWMDecoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
