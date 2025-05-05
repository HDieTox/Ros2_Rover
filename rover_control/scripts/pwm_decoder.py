#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from gpiozero import Button
from time import time

class PWMDecoder(Node):
    def __init__(self):
        super().__init__('pwm_decoder')

        # Configuration des broches
        self.pwm_pins = [17, 18] # GPIO17 et GPIO18
        self.pwm_values = [0, 0]
        self.last_rise = [0, 0]

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
        self.get_logger().info(f"Rising edge detected on channel {channel}")
        self.last_rise[channel] = time()

    def on_falling(self, channel):
        self.get_logger().info(f"Falling edge detected on channel {channel}")
        if self.last_rise[channel] != 0:
            pulse_width = (time() - self.last_rise[channel]) * 1_000_000  # microsecondes
            # Clamp à la plage int16 pour éviter les erreurs de conversion
            pulse_width = max(-32768, min(32767, int(pulse_width)))
            self.pwm_values[channel] = pulse_width

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
