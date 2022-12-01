import pigpio
import time

PWM_CONTROL_PIN = 13
PWM_FREQ = 50
STEP = 10

pi = pigpio.pi()

# back
pi.hardware_PWM(PWM_CONTROL_PIN, PWM_FREQ, 30000)
time.sleep(5)

# forward
pi.hardware_PWM(PWM_CONTROL_PIN, PWM_FREQ, 120000)
time.sleep(5)

pi.set_mode(PWM_CONTROL_PIN, pigpio.INPUT)
        
    
    