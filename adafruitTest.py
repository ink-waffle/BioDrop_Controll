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
speed = np.array([0,
                  0,
                  0], dtype=np.float32)
position = np.array([0,
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
ox = np.array([1, 0, 0], dtype=np.float32)
ox -= np.dot(ox, gravity_normalized) * gravity_normalized
ox /= np.linalg.norm(ox)
oy = np.cross(ox, gravity_normalized)

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
    dRotation = np.where(np.less_equal(np.abs(dRotation), np.float32(0.05)), 0, dRotation)
    dT = np.float32(perf_counter()) - lasttime

    gravity_normalized_mid = gravity_normalized + (np.cross(-dRotation, gravity_normalized) * dT / 2)
    ox_mid = ox + (np.cross(-dRotation, ox) * dT / 2)
    oy_mid = ox + (np.cross(-dRotation, oy) * dT / 2)

    gravity_normalized += np.cross(-dRotation, gravity_normalized_mid) * dT
    ox += np.cross(-dRotation, ox_mid) * dT
    oy += np.cross(-dRotation, oy) * dT

    gravity_normalized /= np.linalg.norm(gravity_normalized)
    ox /= np.linalg.norm(ox)
    oy /= np.linalg.norm(oy)

    gravity = gravity_normalized * gravity_magnitude
    acceleration = np.array(mpu.acceleration) - gravity
    acceleration = np.array([np.dot(acceleration, ox),
                             np.dot(acceleration, oy),
                             np.dot(acceleration, gravity_normalized)])
    speed += acceleration * dT
    position += speed * dT

    lasttime = np.float32(perf_counter())
    # sys.stdout.write(f'\rgX_: {gravity[0]:.2f}, gY_: {gravity[1]:.2f}, gZ_: {gravity[2]:.2f} ; gX: {acceleration[0]:.2f}, gY: {acceleration[1]:.2f}, gZ: {acceleration[2]:.2f}     ')
    sys.stdout.write(f'\raX: {acceleration[0]:.2f}, aY: {acceleration[1]:.2f}, aZ: {acceleration[2]:.2f} ; vX: {speed[0]:.2f}, vY: {speed[1]:.2f}, zZ: {speed[2]:.2f} ; pX: {position[0]:.2f}, pY: {position[1]:.2f}, pZ: {position[2]:.2f}       ')
    sys.stdout.flush()
    sleep(0.01)
