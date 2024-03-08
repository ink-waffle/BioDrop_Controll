import numpy as np
import serial
import adafruit_gps
from time import sleep
import board
import adafruit_mpu6050
import numpy
import sys
from time import process_time_ns

uart = serial.Serial("/dev/serial0", baudrate=9600)
i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)
gps = adafruit_gps.GPS(uart, debug=False)

rotation = np.array([[0.0],
                     [0.0],
                     [0.0]])

noise = np.array([[0.0],
                  [0.0],
                  [0.0]])

lasttime = 0
for i in range(200):
    noise = 0.05 * np.array(mpu.gyro).reshape((3, 1)) + 0.95 * noise
    sleep(0.01)

lasttime = process_time_ns()
while True:
    drotation = np.array(mpu.gyro).reshape((3, 1))
    # noise = 0.01 * drotation + 0.99 * noise
    drotation = drotation - noise

    # rotation = rotation * 0.999
    dT = process_time_ns() - lasttime
    rotation += drotation * dT * 0.000000001
    lasttime = process_time_ns()
    print_rotation = np.round(rotation, 2)
    print_drotation = np.round(drotation, 2)
    sys.stdout.write(f'\rX: {print_rotation[0,0]}, Y: {print_rotation[1,0]}, Z: {print_rotation[2,0]} rad dT: {dT} ns')
    sys.stdout.flush()
