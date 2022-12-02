import pigpio
import time

BLANCE_MOTOR_PIN = 12
BLANCE_MOTOR_PWM_FREQ = 8000

pi = pigpio.pi()

pi.hardware_PWM(BLANCE_MOTOR_PIN, BLANCE_MOTOR_PWM_FREQ, 900000)
time.sleep(1)
pi.hardware_PWM(BLANCE_MOTOR_PIN, BLANCE_MOTOR_PWM_FREQ, 800000)
time.sleep(1)
pi.hardware_PWM(BLANCE_MOTOR_PIN, BLANCE_MOTOR_PWM_FREQ, 700000)
time.sleep(1)
pi.hardware_PWM(BLANCE_MOTOR_PIN, BLANCE_MOTOR_PWM_FREQ, 600000)
time.sleep(1)
pi.hardware_PWM(BLANCE_MOTOR_PIN, BLANCE_MOTOR_PWM_FREQ, 500000)
time.sleep(1)
pi.set_mode(BLANCE_MOTOR_PIN, pigpio.OUTPUT)
