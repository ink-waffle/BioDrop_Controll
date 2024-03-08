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

drotation = np.array([[0],
                     [0],
                     [0]])
while True:
    gyro_readings = np.array(mpu.gyro).reshape((3, 1))
    drotation = 0.05 * gyro_readings + 0.95 * drotation
    drotation = gyro_readings - drotation

    rotation = rotation * 0.997
    rotation += drotation * 0.01
    print_rotation = np.round(rotation, 2)
    print_drotation = np.round(drotation, 2)
    sys.stdout.write(f'\rX: {print_rotation[0,0]}, Y: {print_rotation[1,0]}, Z: {print_rotation[2,0]} rad dX: {print_drotation[0,0]}, dY: {print_drotation[1,0]}, dZ: {print_drotation[2,0]} rad/s')
    sys.stdout.flush()
    sleep(0.01)
