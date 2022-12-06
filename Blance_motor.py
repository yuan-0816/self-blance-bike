# import pigpio
# import time
# 
# BLANCE_MOTOR_PIN = 12
# BLANCE_MOTOR_DIRECTION = 6
# BLANCE_MOTOR_BRAKE = 5
# 
# BLANCE_MOTOR_PWM_FREQ = 8000
# 
# pi = pigpio.pi()
# 
# c = 0
# 
# while 1:
#     pi.write(BLANCE_MOTOR_BRAKE, 0)
#     time.sleep(1)
#     c += 1
#     if c > 5:
#         break
# pi.write(BLANCE_MOTOR_BRAKE, 1)
#     
# # pi.hardware_PWM(BLANCE_MOTOR_PIN, BLANCE_MOTOR_PWM_FREQ, 900000)
# # time.sleep(1)
# # pi.hardware_PWM(BLANCE_MOTOR_PIN, BLANCE_MOTOR_PWM_FREQ, 800000)
# # time.sleep(1)
# # pi.hardware_PWM(BLANCE_MOTOR_PIN, BLANCE_MOTOR_PWM_FREQ, 700000)
# # time.sleep(1)
# # pi.hardware_PWM(BLANCE_MOTOR_PIN, BLANCE_MOTOR_PWM_FREQ, 600000)
# # time.sleep(1)
# # pi.hardware_PWM(BLANCE_MOTOR_PIN, BLANCE_MOTOR_PWM_FREQ, 500000)
# # time.sleep(1)
# # pi.hardware_PWM(BLANCE_MOTOR_PIN, BLANCE_MOTOR_PWM_FREQ, 1000000)
# 
# # pi.set_mode(BLANCE_MOTOR_PIN, pigpio.OUTPUT)
# # pi.write(BLANCE_MOTOR_PIN, 1)
#



print(round(1, 6))