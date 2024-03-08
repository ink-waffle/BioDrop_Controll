import numpy as np
import serial
import adafruit_gps
from time import sleep
import board
import adafruit_mpu6050
import numpy
import sys

uart = serial.Serial("/dev/serial0", baudrate=9600)
i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)
gps = adafruit_gps.GPS(uart, debug=False)

rotation = np.array([[0],
                     [0],
                     [0]])

while True:
    drotation = np.array(mpu.gyro).transpose()
    print(drotation)
    sleep(0.01)
