import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Servo
from time import sleep

class motor_controller(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Paramètres PWM pour les entrées RC du TReX Jr (en secondes)
        # Généralement, 1ms = marche arrière maxi, 1.5ms = stop, 2ms = avant maxi
        self.servo_min_pulse = 1/1000      # 1 ms
        self.servo_max_pulse = 2/1000      # 2 ms

        # Broches GPIO pour les signaux RC (adapter selon ton câblage)
        self.left_pwm_pin = 22
        self.right_pwm_pin = 23

        # Création des objets Servo gpiozero
        self.left_servo = Servo(self.left_pwm_pin, min_pulse_width=self.servo_min_pulse, max_pulse_width=self.servo_max_pulse)
        self.right_servo = Servo(self.right_pwm_pin, min_pulse_width=self.servo_min_pulse, max_pulse_width=self.servo_max_pulse)

        # Souscription au topic /cmd_vel_manual
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_manual',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info("TReX Jr PWM controller node started")

    def cmd_vel_callback(self, msg):
        # Calcul des vitesses gauche/droite [-1, 1]
        left = msg.linear.x - msg.angular.z
        right = msg.linear.x + msg.angular.z

        # Clamp les valeurs dans [-1, 1]
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        # Appliquer les vitesses aux sorties PWM RC
        self.left_servo.value = left
        self.right_servo.value = right

        self.get_logger().info(f"PWM OUT: left={left:.2f}, right={right:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = motor_controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Met les PWM à neutre à l'arrêt
        node.left_servo.value = 0
        node.right_servo.value = 0
        sleep(0.5)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
