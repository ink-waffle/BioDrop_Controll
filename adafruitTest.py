import serial
import adafruit_gps
from time import sleep, perf_counter
import board
import sys
import adafruit_mpu6050
import numpy as np
import sys

uart = serial.Serial("/dev/serial0", baudrate=9600)
i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)
gps = adafruit_gps.GPS(uart, debug=False)

noise = np.array([0,
                  0,
                  0], dtype=np.float32)
gravity = np.array([0,
                    0,
                    0], dtype=np.float32)

for i in range(1000):
    noise += np.float32(0.001) * np.array(mpu.gyro)
    gravity += np.float32(0.001) * np.array(mpu.acceleration)
    sleep(0.001)


# Infer Rotation
print_gravity = np.round(gravity, 2)
gravity_magnitude = np.linalg.norm(gravity)
gravity_normalized = gravity / gravity_magnitude

lasttime = np.float32(perf_counter())
while True:
    # # Update GPS data
    # gps.update()
    #
    # # Check if there are new coordinates available
    # if gps.has_fix:
    #     # Print the position
    #     print('Latitude: {0:.6f}'.format(gps.latitude), 'Longitude: {0:.6f}'.format(gps.longitude))
    # else:
    #     # print("no gps fix")
    #     pass
    dRotation = np.array(mpu.gyro, dtype=np.float32) - noise
    dRotation = np.where(np.less_equal(np.abs(dRotation), np.float32(0.01)), 0, dRotation)
    dT = np.float32(perf_counter()) - lasttime
    # gravity += np.cross(dRotation, gravity) * dT

    print_gravity = np.round(gravity, 2)
    print_angular = np.round(dRotation, 2)
    # sys.stdout.write(f'\rgX: {print_gravity[0]}, gY: {print_gravity[1]}, gZ: {print_gravity[2]}       ')
    sys.stdout.write(f'\rwX: {print_angular[0]}, wY: {print_angular[1]}, wZ: {print_angular[2]}       ')
    sys.stdout.flush()
    sleep(0.1)
