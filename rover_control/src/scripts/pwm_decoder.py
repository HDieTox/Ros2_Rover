from gpiozero import Button
from time import time, sleep
import threading

# Configuration des broches GPIO
PWM_PIN1 = 17
PWM_PIN2 = 18

# Variables globales pour stocker les valeurs PWM (en microsecondes)
pwm_values = [0, 0]
last_rise = [0, 0]

def make_pwm_callback(channel):
    def on_rising():
        last_rise[channel] = time()
    def on_falling():
        if last_rise[channel] != 0:
            pulse_width = (time() - last_rise[channel]) * 1_000_000  # en µs
            pwm_values[channel] = int(pulse_width)
    return on_rising, on_falling

# Instanciation des boutons (utilisés comme entrées numériques)
btn1 = Button(PWM_PIN1, pull_up=False, bounce_time=None)
btn2 = Button(PWM_PIN2, pull_up=False, bounce_time=None)

rising1, falling1 = make_pwm_callback(0)
rising2, falling2 = make_pwm_callback(1)

btn1.when_pressed = rising1
btn1.when_released = falling1
btn2.when_pressed = rising2
btn2.when_released = falling2

def publisher():
    while True:
        print(f"PWM1: {pwm_values[0]} us, PWM2: {pwm_values[1]} us")
        sleep(0.02)  # 20 ms

if __name__ == "__main__":
    pub_thread = threading.Thread(target=publisher, daemon=True)
    pub_thread.start()
    try:
        while True:
            sleep(1)
    except KeyboardInterrupt:
        pass
