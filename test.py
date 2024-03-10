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

# noise_i = np.float64(0)
# noise_j = np.float64(0)
noise = np.array([[0],
                  [0],
                  [0]], dtype=np.float64)
gravity = np.array([[0.0],
                  [0.0],
                  [0.0]])

for i in range(1000):
    # gyro_read = mpu.gyro
    # noise_i += np.float64(0.001) * gyro_read[0]
    # noise_j += np.float64(0.001) * gyro_read[1]
    noise += np.float64(0.001) * np.array(mpu.gyro).reshape((3, 1))
    gravity += np.float64(0.001) * np.array(mpu.acceleration).reshape((3, 1))
    sleep(0.001)


# Infer Rotation
print_gravity = np.round(gravity, 2)
gravity_magnitude = np.linalg.norm(gravity)
gravity_normalized = gravity / gravity_magnitude

# [[cosk * -sinj],        = [[G_x],
#  [cosk * -sini * cosj + sinj * sink], =  [G_y],
#  [cosi * cosj]]  =  [G_z]]
initialPitch = pitch = np.float64(np.arcsin(-gravity_normalized[0, 0]))
initialRoll = roll = np.float64(np.arcsin(gravity_normalized[1, 0] / (np.cos(pitch))))
initialYawn = yawn = np.float64(0)

print(f'roll: {np.round(initialRoll, 2)} pitch: {np.round(initialPitch, 2)}')
print(f'gravity magnitude: {np.round(gravity_magnitude, 2)}')
print(f'gX: {print_gravity[0,0]}, gY: {print_gravity[1,0]}, gZ: {print_gravity[2,0]};')
# Done Rotation Inference


lasttime = perf_counter()
while True:
    # di, dj, _ = mpu.gyro
    # noise = 0.01 * drotation + 0.99 * noise
    # noise_i = np.float64(0.0001) * np.float64(di) + np.float64(0.9999) * np.float64(noise_i)
    # noise_j = np.float64(0.0001) * np.float64(dj) + np.float64(0.9999) * np.float64(noise_j)
    dRotation = np.array(mpu.gyro, dtype=np.float64).reshape((3, 1)) - noise
    dRotation = np.where(np.less_equal(np.abs(dRotation), np.float64(0.01)), 0, dRotation)

    dT = np.float64(perf_counter() - lasttime)
    dRotation *= dT
    roll += dRotation[0, 0]
    pitch += dRotation[1, 0]
    yawn += dRotation[2, 0]
    lasttime = perf_counter()

    roll = roll - np.float64(2) if np.greater_equal(roll, np.float64(2)) else roll
    pitch = pitch - np.float64(2) if np.greater_equal(pitch, np.float64(2)) else pitch
    yawn = yawn - np.float64(2) if np.greater_equal(yawn, np.float64(2)) else yawn
    roll = roll + np.float64(2) if np.less_equal(roll, np.float64(-2)) else roll
    pitch = pitch + np.float64(2) if np.less_equal(pitch, np.float64(-2)) else pitch
    yawn = yawn + np.float64(2) if np.less_equal(yawn, np.float64(-2)) else yawn

    roll += np.float64(0.000002) * (initialRoll - roll)
    pitch += np.float64(0.000002) * (initialPitch - pitch)
    yawn += np.float64(0.000002) * (initialYawn - yawn)

    gravity = gravity_magnitude * np.array([[np.cos(yawn) * -np.sin(pitch)],
                                            [np.cos(yawn) * np.sin(roll) * np.cos(pitch) + np.sin(pitch) * np.sin(yawn)],
                                            [np.cos(roll) * np.cos(pitch)]])
    print_gravity = np.round(gravity, 2)
    print_acceleration = np.round(np.array(mpu.acceleration), 2)
    sys.stdout.write(f'\raX: {print_acceleration[0]}, aY: {print_acceleration[1]}, aZ: {print_acceleration[2]}; gX: {print_gravity[0,0]}, gY: {print_gravity[1,0]}, gZ: {print_gravity[2,0]}; roll: {np.round(roll, 2)}, pitch: {np.round(pitch, 2)}, yawn: {np.round(yawn, 2)}  ')
    sys.stdout.flush()
    sleep(0.0001)
