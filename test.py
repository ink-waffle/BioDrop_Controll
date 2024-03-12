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

noise = np.array([0,
                  0,
                  0], dtype=np.float32)
gravity = np.array([0.0,
                    0.0,
                    0.0])

for i in range(1000):
    noise += np.float32(0.001) * np.array(mpu.gyro)
    gravity += np.float32(0.001) * np.array(mpu.acceleration)
    sleep(0.001)


# Infer Rotation
print_gravity = np.round(gravity, 2)
gravity_magnitude = np.linalg.norm(gravity)
gravity_normalized = gravity / gravity_magnitude

# [sink * -sini * cosj + cosk * -sinj] = [G_x]
# [cosk * -sini * cosj + sinj * sink] == [G_y]
# [cosi * cosj] ======================== [G_z]
# [cosi * sink] = [Y_x]
# [cosi * cosk] = [Y_y]
# [-sini] ======= [Y_z]
initialPitch = pitch = np.float32(np.arcsin(-gravity_normalized[0]))
initialRoll = roll = np.float32(np.arcsin(gravity_normalized[1] / (np.cos(pitch))))
initialYawn = yawn = np.float32(0)

z_vector = np.array([np.sin(yawn) * -np.sin(roll) * np.cos(pitch) + np.cos(yawn) * -np.sin(pitch),
                     np.cos(yawn) * np.sin(roll) * np.cos(pitch) + np.sin(pitch) * np.sin(yawn),
                     np.cos(roll) * np.cos(pitch)])
y_vector = np.array([np.cos(roll) * np.sin(yawn),
                     np.cos(roll) * np.cos(yawn),
                     -np.sin(roll)])
x_vector = np.array([y_vector[1] * z_vector[2] - y_vector[2] * z_vector[1],
                     y_vector[2] * z_vector[0] - y_vector[0] * z_vector[2],
                     y_vector[0] * z_vector[1] - y_vector[1] * z_vector[0]])

gravity = gravity_magnitude * z_vector
print_y = np.round(y_vector, 2)
print(f'roll: {np.round(initialRoll, 2)} pitch: {np.round(initialPitch, 2)}')
print(f'gravity magnitude: {np.round(gravity_magnitude, 2)}')
print(f'gX: {print_gravity[0]}, gY: {print_gravity[1]}, gZ: {print_gravity[2]};')
print(f'yX: {print_y[0]}, yY: {print_y[1]}, yZ: {print_y[2]};')
# Done Rotation Inference

speed = np.array([0, 0, 0], dtype=np.float32)
traversedDistance = np.float32(0)
lasttime = np.float32(perf_counter())
while True:
    # noise = 0.01 * drotation + 0.99 * noise
    dRotation = np.array(mpu.gyro, dtype=np.float32) - noise
    dRotation = np.where(np.less_equal(np.abs(dRotation), np.float32(0.01)), 0, dRotation)
    dRotation = np.array([[1, np.sin(roll) * np.tan(pitch), np.cos(roll) * np.tan(pitch)],
                          [0, np.cos(roll), -np.sin(roll)],
                          [0, np.sin(roll)/np.cos(pitch), np.cos(roll)/np.cos(pitch)]]) @ dRotation

    dT = np.float32(perf_counter()) - lasttime
    dRotation *= dT
    roll += dRotation[0]
    pitch += dRotation[1]
    yawn += dRotation[2]
    lasttime = np.float32(perf_counter())


    roll += np.float32(0.000003) if initialRoll > roll else np.float32(-0.000003)
    pitch += np.float32(0.000003) if initialPitch > pitch else np.float32(-0.000003)
    yawn += np.float32(0.000003) if initialYawn > yawn else np.float32(-0.000003)

    z_vector = np.array([np.sin(yawn) * -np.sin(roll) * np.cos(pitch) + np.cos(yawn) * -np.sin(pitch),
                         np.cos(yawn) * np.sin(roll) * np.cos(pitch) + np.sin(pitch) * np.sin(yawn),
                         np.cos(roll) * np.cos(pitch)])
    y_vector = np.array([np.cos(roll) * np.sin(yawn),
                         np.cos(roll) * np.cos(yawn),
                         -np.sin(roll)])
    x_vector = np.array([y_vector[1] * z_vector[2] - y_vector[2] * z_vector[1],
                         y_vector[2] * z_vector[0] - y_vector[0] * z_vector[2],
                         y_vector[0] * z_vector[1] - y_vector[1] * z_vector[0]])

    gravity = gravity_magnitude * z_vector
    acceleration = np.array(mpu.acceleration) - gravity

    acceleration_projection_x = np.dot(acceleration, x_vector)
    acceleration_projection_y = np.dot(acceleration, y_vector)
    acceleration_projection_z = np.dot(acceleration, z_vector)

    speed[0] += acceleration_projection_x * dT
    speed[1] += acceleration_projection_y * dT
    speed[2] += acceleration_projection_z * dT
    traversedDistance += np.linalg.norm(speed) * dT

    # print_gravity = np.round(gravity, 2)
    # print_acceleration = np.round(np.array(mpu.acceleration), 2)
    print_x = np.int16(x_vector * 100)
    print_y = np.int16(y_vector * 100)
    print_acceleration = np.int16(acceleration * 100)
    print_speed = np.int16(speed * 100)
    # sys.stdout.write(f'\rvX: {print_speed[0]}, vY: {print_speed[1]}, vZ: {print_speed[2]}; roll: {np.round(roll, 2)}, pitch: {np.round(pitch, 2)}, yawn: {np.round(yawn, 2)}; dist: {np.int16(traversedDistance * 100)}  ')
    sys.stdout.write(f'\rxX: {print_x[0]}, xY: {print_x[1]}, xZ: {print_x[2]}; yX: {print_y[0]}, yY: {print_y[1]}, yZ: {print_y[2]}; ')
    sys.stdout.flush()
    sleep(0.01)
