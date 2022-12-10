import keyboard
import pigpio
import time


REAR_WHEEl_PIN = 16   # MG996R Sofe PWM


PWM_FREQ = 50
SPEED = 0.001
STEP = 10
BUFFER_TIME = 0.07

pi = pigpio.pi()

pi.set_PWM_frequency(REAR_WHEEl_PIN, PWM_FREQ)


try:
    while 1:
        pi.set_PWM_dutycycle(REAR_WHEEl_PIN, 0)
#         pi.hardware_PWM(REAR_WHEEl_PIN, PWM_FREQ, 0)        
        if keyboard.is_pressed('up'):
#             pi.hardware_PWM(REAR_WHEEl_PIN, PWM_FREQ, 120000)
            pi.set_PWM_dutycycle(REAR_WHEEl_PIN, int(0.12*256))    # dutycycle = 12%
            time.sleep(0.05)
            
        if keyboard.is_pressed('down'):
#             pi.hardware_PWM(REAR_WHEEl_PIN, PWM_FREQ, 30000)
            pi.set_PWM_dutycycle(REAR_WHEEl_PIN, int(0.03*256))    # dutycycle = 3%
            time.sleep(0.05)

finally:
    pi.set_mode(REAR_WHEEl_PIN, pigpio.INPUT)
        

        

