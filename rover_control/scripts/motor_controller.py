#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import PWMOutputDevice
from time import sleep

class motor_controller(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Broches GPIO pour PWM software (adapter selon ton câblage)
        self.left_pwm_pin = 22
        self.right_pwm_pin = 23

        # Création des objets PWMOutputDevice (PWM software)
        self.left_pwm = PWMOutputDevice(self.left_pwm_pin, frequency=50)  # 50 Hz standard servo freq
        self.right_pwm = PWMOutputDevice(self.right_pwm_pin, frequency=50)

        # Souscription au topic /cmd_vel_manual
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_manual',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info("motor_controller node started")

    def cmd_vel_callback(self, msg):
        # Calcul des vitesses gauche/droite [-1, 1]
        left = msg.linear.x - msg.angular.z
        right = msg.linear.x + msg.angular.z

        # Clamp dans [-1, 1]
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        # Convertir de [-1,1] à [0,1] duty cycle
        left_duty = (left + 1) / 2
        right_duty = (right + 1) / 2

        # Appliquer le duty cycle PWM software
        self.left_pwm.value = left_duty
        self.right_pwm.value = right_duty

        self.get_logger().info(f"PWM OUT (soft): left duty={left_duty:.2f}, right duty={right_duty:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = motor_controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.left_pwm.off()
        node.right_pwm.off()
        sleep(0.5)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
