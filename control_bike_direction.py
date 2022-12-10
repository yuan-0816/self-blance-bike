import keyboard
import pigpio
import time


DIRECTION_PIN = 19    # MG90S Hardware PWM

PWM_FREQ = 50
SPEED = 0.001
STEP = 10
BUFFER_TIME = 0.07

pi = pigpio.pi()

def angle_to_duty_cycle(angle = 0):
    duty_cycle = int((500 * PWM_FREQ + (1900 * PWM_FREQ * angle / 180)))
    return duty_cycle

# pi.set_mode(DIRECTION_PIN, pigpio.INPUT)

try:
    while 1:
        pi.hardware_PWM(DIRECTION_PIN, PWM_FREQ, angle_to_duty_cycle(90))
#         pi.hardware_PWM(REAR_WHEEl_PIN, PWM_FREQ, 0)
            
        if keyboard.is_pressed('right'):
            for angle in range(90, 121, STEP):
                pi.hardware_PWM(DIRECTION_PIN, PWM_FREQ, angle_to_duty_cycle(angle))
                time.sleep(SPEED)
            time.sleep(BUFFER_TIME)
        if keyboard.is_pressed('left'):
            for angle in range(90, 59, -STEP):
                pi.hardware_PWM(DIRECTION_PIN, PWM_FREQ, angle_to_duty_cycle(angle))
                time.sleep(SPEED)
            time.sleep(BUFFER_TIME)

finally:
    pi.set_mode(DIRECTION_PIN, pigpio.INPUT)

        

        
