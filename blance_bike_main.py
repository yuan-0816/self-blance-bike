import pigpio
import time
import smbus #import SMBus module of I2C
import numpy as np
import math
from Kalman import Kalman

DEBUG = False
SLEEP_TIME = 1    # main loop sleep [ms]

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A    
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

# DLPF frequency
FILTER_BW_256 = 0x00
FILTER_BW_188 = 0x01
FILTER_BW_98 = 0x02
FILTER_BW_42 = 0x03
FILTER_BW_20 = 0x04
FILTER_BW_10 = 0x05
FILTER_BW_5 = 0x06
Device_Address = 0x68   # MPU6050 device address

GRAVITIY = 9.80665

CALIBRATE_SIZE = 1500

BLANCE_MOTOR_PWM_FREQ = 20000
BLANCE_MOTOR_PIN = 12
BLANCE_MOTOR_DIRECTION = 6
BLANCE_MOTOR_BRAKE = 5    # pi.write(BLANCE_MOTOR_BRAKE, 0) is brake

ANGLE_LIMIT = 13
TARGET_ANGLE = -2
Angle_FIXRATE = 0
KP = 0
KI = 0
KD = 0
error = 0
integral = 0
derivative = 0
prevError = 0
PIDoutput = 0
motorCtrl = 0
kal_deg = 0
deg = 0

# kal_deg, PIDoutput, targetAngle, error
logData = [["kal_deg", "targetAngle", "error"]]

def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    #Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    #Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)
    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    #Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)
    
    # this is DLPF
def set_filter_range(self, filter_range=FILTER_BW_256):
    """Sets the low-pass bandpass filter frequency"""
    # Keep the current EXT_SYNC_SET configuration in bits 3, 4, 5 in the MPU_CONFIG register
    EXT_SYNC_SET = bus.read_byte_data(Device_Address, CONFIG) & 0b00111000
    return bus.write_byte_data(Device_Address, CONFIG, EXT_SYNC_SET | filter_range)

def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)
    #concatenate higher and lower value
    value = ((high << 8) | low)
    #to get signed value from mpu6050
    if(value > 32768):
            value = value - 65536
    return value

def read_data_tuple(data=None):
    if data != "acc" and data != "gyr":
        print("Please input the argument!")
        return
    elif data == "acc":
        return read_raw_data(ACCEL_XOUT_H), read_raw_data(ACCEL_YOUT_H), read_raw_data(ACCEL_ZOUT_H)
    elif data == "gyr":
        return read_raw_data(GYRO_XOUT_H), read_raw_data(GYRO_YOUT_H), read_raw_data(GYRO_ZOUT_H)
        
def RectifyData(CALIBRATE_SIZE):
    print("MPU6050 is calibrating - keep the MPU6050 steady...")
    acc_array = []    # [x, y, z]
    gyr_array = []
    acc_offsets = [0.0, 0.0, 0.0]
    gyr_offsets = [0.0, 0.0, 0.0]
    while True:
        try:
            ax, ay, az = read_data_tuple("acc")
            gx, gy, gz = read_data_tuple("gyr")
#             print(ax, ay, az)
#             print(gx, gy, gz)
        except:
            continue    
        acc_array.append([ax, ay, az])
        gyr_array.append([gx, gy, gz])
#         print(np.shape(acc_array)[0])
        print("\n---------------------------------------------------")
        print("MPU6050 is calibrating - keep the MPU6050 steady...")
        print("This is " + str(np.shape(acc_array)[0])+ " times")
        if np.shape(acc_array)[0] == CALIBRATE_SIZE:
            for i in range(0, 3):
                acc_offsets[i] = np.mean(np.array(acc_array)[:, i])
                gyr_offsets[i] = np.mean(np.array(gyr_array)[:, i])
            break
    print("MPU6050 is calibrated")
    return acc_offsets, gyr_offsets

def OffSet_Data(data_type, datas):
    if data_type != "acc" and data_type != "gyr":
        print("Please input data type!")
        return
    elif data_type == "acc":
        datas[0] = datas[0] - acc_offsets[0] + (-16384)  #1g(m/s^2)
        datas[1] = datas[1] - acc_offsets[1]
        datas[2] = datas[2] - acc_offsets[2]
        return datas
    elif data_type == "gyr":
        datas[0] = datas[0] - gyr_offsets[0]
        datas[1] = datas[1] - gyr_offsets[1]
        datas[2] = datas[2] - gyr_offsets[2]
        return datas
    
def dist(a, b):
    return math.sqrt((a*a)+(b*b))
       
def get_z_angle(x, y, z):    # right is positive, left is negative
    radians = math.atan2(y, dist(x, z))
    return math.degrees(radians)

bus = smbus.SMBus(1) # or bus = smbus.SMBus(0) for older version boards
MPU_Init()

#set DLPF filters 
set_filter_range(FILTER_BW_5)

if (DEBUG):
    acc_offsets, gyr_offsets = RectifyData(CALIBRATE_SIZE)
    print(acc_offsets, gyr_offsets)

acc_offsets = [-15775.053333333333, -84.50666666666666, -2522.866666666667]    #[accX, accY, accZ]
gyr_offsets = [8.858666666666666, -4.046666666666667, 15.742666666666667]    #[gyrX, gyrY, gyrZ]

kalAngleY = Kalman()
timer = time.time()

pi = pigpio.pi()

pi.write(BLANCE_MOTOR_BRAKE, 1)

if __name__ == "__main__":
    try:
        if not DEBUG:
            while True:
                #Read Accelerometer raw value
                acc = [read_raw_data(ACCEL_XOUT_H),
                       read_raw_data(ACCEL_YOUT_H),
                       read_raw_data(ACCEL_ZOUT_H)]
                #Read Gyroscope raw value
                gyr = [read_raw_data(GYRO_XOUT_H),
                       read_raw_data(GYRO_YOUT_H),
                       read_raw_data(GYRO_ZOUT_H)]
                
                dt = time.time() - timer
                timer = time.time()
                
                # calibrate
                cal_acc = OffSet_Data(data_type="acc", datas=acc)
                cal_gyr = OffSet_Data(data_type="gyr", datas=gyr)
               
                # Full scale range +/- 250 degree/C as per sensitivity scale factor
                for i in range(0, 3):
                    cal_acc[i] /= 16384.0
                    cal_gyr[i] /= 131.072
                deg = get_z_angle(cal_acc[0], cal_acc[1], cal_acc[2])
                kal_deg = kalAngleY.getAngle(deg, cal_gyr[2], dt)
                print(kal_deg, TARGET_ANGLE)
#                 print(dt)
                
                # safety check
                if (abs(kal_deg) >= ANGLE_LIMIT):
                    pi.write(BLANCE_MOTOR_BRAKE, 0)
                    break
                
                # variate target angle
#                 Angle_FIXRATE = 1
#                 if kal_deg < TARGET_ANGLE:
#                     TARGET_ANGLE += Angle_FIXRATE * dt
#                 else:
#                     TARGET_ANGLE -= Angle_FIXRATE * dt
                
                # PID cotrol
                KP = 0.179
                KI = 0.1
                KD = 0.05
                error = TARGET_ANGLE - kal_deg
                integral += error * dt
                derivative = (error - prevError) / dt
                prevError = error
                PIDoutput = KP * error + KI * integral + KD * derivative
#                 print(PIDoutput, kal_deg)
                motorCtrl = min(max(PIDoutput, -1), 1)
#                 print(PIDoutput, motorCtrl)
#                 print(int(round(abs(1-motorCtrl), 6) * 1000000))
#                 print(motorCtrl)
                motor_pwm = (int(round(1-abs(motorCtrl), 6) * 1000000))
#                 print(motor_pwm/10000)
                pi.hardware_PWM(BLANCE_MOTOR_PIN, BLANCE_MOTOR_PWM_FREQ, int(motor_pwm))
                if motorCtrl > 0:
                    pi.write(BLANCE_MOTOR_DIRECTION, 0)
                if motorCtrl < 0:
                    pi.write(BLANCE_MOTOR_DIRECTION, 1)

                logData.append([kal_deg, TARGET_ANGLE, error])

                
#                 time.sleep(0.05)
                time.sleep(SLEEP_TIME / 1000)
    finally:
        pi.write(BLANCE_MOTOR_BRAKE, 0)
        pi.hardware_PWM(BLANCE_MOTOR_PIN, BLANCE_MOTOR_PWM_FREQ, 1000000)


#write log data to file
print("log size", len(logData), "rows")
if len(logData) > 0:
    filename = "datalog.dat"
    #filename = "datalog_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".dat"
    print("write to file:", filename)
    file = open(filename, "w")
    for logLine in logData:
        for value in logLine:
            file.write(str(value) + ' ')
        file.write('\n')
    file.close()

print("program ended")
