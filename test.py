import numpy as np
import serial
import adafruit_gps
from time import sleep
import board
import adafruit_mpu6050
import numpy
import sys
from time import perf_counter

uart = serial.Serial("/dev/serial0", baudrate=9600)
i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)
gps = adafruit_gps.GPS(uart, debug=False)

noise_i = 0.0
noise_j = 0.0
gravity = np.array([[0.0],
                  [0.0],
                  [0.0]])

for i in range(200):
    gyro_read = mpu.gyro
    noise_i += 0.005 * gyro_read[0]
    noise_j += 0.005 * gyro_read[1]
    gravity += 0.005 * np.array(mpu.acceleration).reshape((3, 1))
    sleep(0.005)


# Infer Rotation
gravity_magnitude = np.linalg.norm(gravity)
gravity_normalized = gravity / gravity_magnitude

# [[-sinj],        = [[G_x],
#  [-sini * cosj], =  [G_y],
#  [cosi * cosj]]  =  [G_z]]
pitch = np.arcsin(-gravity_normalized[0, 0])
roll = np.arcsin(-gravity_normalized[1, 0] / (np.cos(pitch)))
print(f'roll: {np.round(roll, 2)} pitch: {np.round(pitch, 2)}')
print(f'gravity magnitude: {np.round(gravity_magnitude, 2)}')
# Done Rotation Inference



lasttime = perf_counter()
while True:
    di, dj, _ = mpu.gyro
    # noise = 0.01 * drotation + 0.99 * noise
    di = di - noise_i
    dj = dj - noise_j

    # rotation = rotation * 0.999
    dT = perf_counter() - lasttime
    roll += di * dT
    pitch += dj * dT
    lasttime = perf_counter()

    gravity = gravity_magnitude * np.array([[-np.sin(pitch)],
                                            [-np.sin(roll) * np.cos(pitch)],
                                            [np.cos(roll) * np.cos(pitch)]])
    print_gravity = np.round(gravity, 2)
    print_acceleration = np.round(np.array(mpu.acceleration), 2)
    sys.stdout.write(f'\raX: {print_acceleration[0]}, aY: {print_acceleration[1]}, aZ: {print_acceleration[2]}; gX: {print_gravity[0,0]}, gY: {print_gravity[1,0]}, gZ: {print_gravity[2,0]}; roll: {np.round(roll, 2)} pitch: {np.round(pitch, 2)}   ')
    sys.stdout.flush()
    sleep(0.001)
