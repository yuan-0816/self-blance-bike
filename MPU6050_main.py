import smbus #import SMBus module of I2C
import time   
import math
import numpy as np
from Kalman import Kalman


DEBUG = True

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

GRAVITIY = 9.80665
CALIBRATE_SIZE=1000

Device_Address = 0x68   # MPU6050 device address

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

acc_offsets = [-15694.684, 269.844, -3425.6]    #[accX, accY, accZ]
gyr_offsets = [6.276, -1.806, 14.235]    #[gyrX, gyrY, gyrZ]


kalAngleY = Kalman()
timer = time.time()

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
        
        # acc = [read_data_tuple("acc")]
        # gyr = [read_data_tuple("gyr")]
        # print(acc)
        # print(gyr)
        
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
    #     print(deg)

        kal_deg = kalAngleY.getAngle(deg, cal_gyr[2], dt)
    #     print("gyr:", cal_gyr[2])
        print("kalman:", kal_deg, "deg:", deg)
    #     print(kal_deg)
        
    #     print(cal_gyr[2])
        time.sleep(0.03)




