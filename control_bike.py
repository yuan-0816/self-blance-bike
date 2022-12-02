import keyboard
import pigpio
import time


DIRECTION_PIN = 19    # MG90S Hardware PWM
REAR_WHEEl_PIN = 16   # MG996R Sofe PWM


PWM_FREQ = 50
SPEED = 0.001
STEP = 10
BUFFER_TIME = 0.07

pi = pigpio.pi()

pi.set_PWM_frequency(REAR_WHEEl_PIN, PWM_FREQ)

def angle_to_duty_cycle(angle = 0):
    duty_cycle = int((500 * PWM_FREQ + (1900 * PWM_FREQ * angle / 180)))
    return duty_cycle

# pi.set_mode(DIRECTION_PIN, pigpio.INPUT)

try:
    while 1:
        pi.hardware_PWM(DIRECTION_PIN, PWM_FREQ, angle_to_duty_cycle(90))
        pi.set_PWM_dutycycle(REAR_WHEEl_PIN, 0)
#         pi.hardware_PWM(REAR_WHEEl_PIN, PWM_FREQ, 0)
            
        if keyboard.is_pressed('d'):
            for angle in range(90, 121, STEP):
                pi.hardware_PWM(DIRECTION_PIN, PWM_FREQ, angle_to_duty_cycle(angle))
                time.sleep(SPEED)
            time.sleep(BUFFER_TIME)
        if keyboard.is_pressed('a'):
            for angle in range(90, 59, -STEP):
                pi.hardware_PWM(DIRECTION_PIN, PWM_FREQ, angle_to_duty_cycle(angle))
                time.sleep(SPEED)
            time.sleep(BUFFER_TIME)
            
        if keyboard.is_pressed('w'):
#             pi.hardware_PWM(REAR_WHEEl_PIN, PWM_FREQ, 120000)
            pi.set_PWM_dutycycle(REAR_WHEEl_PIN, int(0.12*256))    # dutycycle = 12%
            time.sleep(0.05)
            
        if keyboard.is_pressed('s'):
#             pi.hardware_PWM(REAR_WHEEl_PIN, PWM_FREQ, 30000)
            pi.set_PWM_dutycycle(REAR_WHEEl_PIN, int(0.03*256))    # dutycycle = 3%
            time.sleep(0.05)

finally:
    pi.set_mode(DIRECTION_PIN, pigpio.INPUT)
    pi.set_mode(REAR_WHEEl_PIN, pigpio.INPUT)
        

        
