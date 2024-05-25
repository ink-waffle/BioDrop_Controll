import datetime
import serial
import adafruit_gps
from time import sleep, perf_counter
import board
import busio
import sys
import adafruit_mpu6050
import numpy as np
import sys
import pandas as pd

uart = serial.Serial("/dev/serial0", baudrate=9600)
i2c_1 = busio.I2C(board.SCL, board.SDA)
i2c_2 = busio.I2C(board.GPIO6, board.GPIO5)
mpu_1 = adafruit_mpu6050.MPU6050(i2c_1)
mpu_2 = adafruit_mpu6050.MPU6050(i2c_2)
while True:
    print(f'mpu_1: {mpu_1.acceleration} ; {mpu_1.gyro}')
    print(f'mpu_2: {mpu_2.acceleration} ; {mpu_2.gyro}')
    sleep(1)