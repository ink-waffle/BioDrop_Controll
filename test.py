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

noise = np.array([[0],
                  [0],
                  [0]], dtype=np.float32)
gravity = np.array([[0.0],
                  [0.0],
                  [0.0]])

for i in range(1000):
    noise += np.float32(0.001) * np.array(mpu.gyro).reshape((3, 1))
    gravity += np.float32(0.001) * np.array(mpu.acceleration).reshape((3, 1))
    sleep(0.001)


# Infer Rotation
print_gravity = np.round(gravity, 2)
gravity_magnitude = np.linalg.norm(gravity)
gravity_normalized = gravity / gravity_magnitude

# [[cosk * -sinj],        = [[G_x],
#  [cosk * -sini * cosj + sinj * sink], =  [G_y],
#  [cosi * cosj]]  =  [G_z]]
initialPitch = pitch = np.float32(np.arcsin(-gravity_normalized[0, 0]))
initialRoll = roll = np.float32(np.arcsin(gravity_normalized[1, 0] / (np.cos(pitch))))
initialYawn = yawn = np.float32(0)

print(f'roll: {np.round(initialRoll, 2)} pitch: {np.round(initialPitch, 2)}')
print(f'gravity magnitude: {np.round(gravity_magnitude, 2)}')
print(f'gX: {print_gravity[0,0]}, gY: {print_gravity[1,0]}, gZ: {print_gravity[2,0]};')
# Done Rotation Inference


lasttime = np.float32(perf_counter())
while True:
    # noise = 0.01 * drotation + 0.99 * noise
    dRotation = np.array(mpu.gyro, dtype=np.float32).reshape((3, 1)) - noise
    dRotation = np.where(np.less_equal(np.abs(dRotation), np.float32(0.01)), 0, dRotation)
    dRotation = np.array([[1, np.sin(roll) * np.tan(pitch), np.cos(roll) * np.tan(pitch)],
                          [0, np.cos(roll), -np.sin(roll)],
                          [0, np.sin(roll)/np.cos(pitch), np.cos(roll)/np.cos(pitch)]]) @ dRotation

    dT = np.float32(perf_counter()) - lasttime
    dRotation *= dT
    roll += dRotation[0, 0]
    pitch += dRotation[1, 0]
    yawn += dRotation[2, 0]
    lasttime = np.float32(perf_counter())


    roll += np.float32(0.000003) if initialRoll > roll else np.float32(-0.000003)
    pitch += np.float32(0.000003) if initialPitch > pitch else np.float32(-0.000003)
    yawn += np.float32(0.000003) if initialYawn > yawn else np.float32(-0.000003)

    gravity = gravity_magnitude * np.array([[np.cos(yawn) * -np.sin(pitch)],
                                            [np.cos(yawn) * np.sin(roll) * np.cos(pitch) + np.sin(pitch) * np.sin(yawn)],
                                            [np.cos(roll) * np.cos(pitch)]])
    acceleration = np.array(mpu.acceleration).reshape((3, 1)) - gravity
    # print_gravity = np.round(gravity, 2)
    # print_acceleration = np.round(np.array(mpu.acceleration), 2)
    print_acceleration = np.round(acceleration, 2)
    sys.stdout.write(f'\raX: {print_acceleration[0, 0]}, aY: {print_acceleration[1, 0]}, aZ: {print_acceleration[2, 0]}; roll: {np.round(roll, 2)}, pitch: {np.round(pitch, 2)}, yawn: {np.round(yawn, 2)}; dT: {dT}  ')
    sys.stdout.flush()
    sleep(0.01)
