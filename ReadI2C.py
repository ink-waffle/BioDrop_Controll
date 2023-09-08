# import smbus
#
# import time
#
# address = 0x48
# A0 = 0x40
# A1 = 0x41
# A2 = 0x42
#
# bus = smbus.SMBus(1)
#
# while True:
#
#     bus.write_byte(address, A2)
#     value = bus.read_byte(address)
#     print("X: " + str(value))
#     time.sleep(0.01)
#
#     bus.write_byte(address, A1)
#     value = bus.read_byte(address)
#     print("Y: " + str(value))
#     time.sleep(0.01)
#
#     bus.write_byte(address, A0)
#     value = bus.read_byte(address)
#     print("Z: " + str(value))
#
#     print('-------------------')
#     time.sleep(0.5)

import time
import board
import busio
import adafruit_adxl34x

i2c = busio.I2C(board.SCL, board.SDA)
accelerometer = adafruit_adxl34x.ADXL345(i2c)

while True:
    print("%f %f %f"%accelerometer.acceleration)
    time.sleep(1)