import spidev as SPI
import time

# register command
CMD_ANGLE = [0xFF, 0xFF]
CMD_AGC = [0x7F, 0xFD]
CMD_MAG = [0x7F, 0xFE]
CMD_CLEAR = [0x40, 0x01]
CMD_NOP = [0xC0, 0x00]

BUS = 0
DEVICE = 0

spi = SPI.SpiDev()
spi.open(BUS, DEVICE)    # /dev/spi-decv0.0
spi.mode = 1    # CPOL=0, CPHA=1
spi.bits_per_word = 8    # AS5048A neesds 16 bits
spi.lsbfirst = False    # MSB first
spi.max_speed_hz = 61000


try:
    while True:

        spi.writebytes(CMD_ANGLE)

#         time.sleep(10/1000000.0) # 10us

        data = spi.readbytes(len(CMD_ANGLE))
#         print(data)
#         print(bin(data[1])[2:].zfill(8)
        # type is str
        result = "0b" + bin(data[0])[2:].zfill(8)[2:] + bin(data[1])[2:].zfill(8)
        angle = int(result, 2)
#         print(bin(angle))
        deg = angle / 16384.0 * 360.0
        print(deg)

        time.sleep(0.05)
finally:
    spi.close()




