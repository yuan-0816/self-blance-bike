import pigpio
import time



REAR_WHEEl_PIN = 16   # MG996R

pi = pigpio.pi()
pi.set_PWM_frequency(REAR_WHEEl_PIN, 50)

try:
    while 1:
        pi.set_PWM_dutycycle(REAR_WHEEl_PIN, int(0.12*256))
        time.sleep(5)
        pi.set_PWM_dutycycle(REAR_WHEEl_PIN, int(0.03*256))
        time.sleep(5)

finally:
    pi.set_mode(REAR_WHEEl_PIN, pigpio.INPUT)

