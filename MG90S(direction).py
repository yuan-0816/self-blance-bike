import pigpio
import time

PWM_CONTROL_PIN = 19
PWM_FREQ = 20
STEP = 30
SPEED = 0.001

pi = pigpio.pi()

def angle_to_duty_cycle(angle = 0):
    duty_cycle = int((500 * PWM_FREQ + (1900 * PWM_FREQ * angle / 180)))
    return duty_cycle

# FORWARD
pi.hardware_PWM(PWM_CONTROL_PIN, PWM_FREQ, angle_to_duty_cycle(90))
time.sleep(1)



# RIGHT
for angle in range(90, 121, 3):
    pi.hardware_PWM(PWM_CONTROL_PIN, PWM_FREQ, angle_to_duty_cycle(angle))
    time.sleep(SPEED)
time.sleep(0.5)
for angle in range(120, 90, -3):
    pi.hardware_PWM(PWM_CONTROL_PIN, PWM_FREQ, angle_to_duty_cycle(angle))
    time.sleep(SPEED)
time.sleep(0.5)

pi.hardware_PWM(PWM_CONTROL_PIN, PWM_FREQ, angle_to_duty_cycle(90))
time.sleep(1)

# LEFT
for angle in range(90, 59, -1):
    pi.hardware_PWM(PWM_CONTROL_PIN, PWM_FREQ, angle_to_duty_cycle(angle))
    time.sleep(SPEED)
time.sleep(0.5)
for angle in range(60, 91, 1):
    pi.hardware_PWM(PWM_CONTROL_PIN, PWM_FREQ, angle_to_duty_cycle(angle))
    time.sleep(SPEED)
time.sleep(0.5)

pi.set_mode(PWM_CONTROL_PIN, pigpio.INPUT)

