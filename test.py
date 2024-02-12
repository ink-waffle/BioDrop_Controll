import numpy as np
import serial
import adafruit_gps
from time import sleep
import board
import adafruit_mpu6050
import numpy
import sys
# Create a serial connection
uart = serial.Serial("/dev/serial0", baudrate=9600)
i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)
# Create a GPS module instance
gps = adafruit_gps.GPS(uart, debug=False)
gravity = np.array([[0],
                    [0],
                    [0]])
filtered_acceleration = np.array([[0],
                                  [0],
                                  [0]])
speed = np.array([[0],
                  [0],
                  [0]], dtype=np.float64)
disposition = np.array([[0],
                        [0],
                        [0]], dtype=np.float64)
alpha = 0.95

for i in range(500):
    acc = mpu.acceleration
    acc = np.array([[acc[0]],
                    [acc[1]],
                    [acc[2]]])
    gravity = alpha * gravity + (1 - alpha) * acc

    filtered_acceleration = acc - gravity
while True:
    # Update GPS data
    gps.update()

    # Check if there are new coordinates available
    if gps.has_fix:
        # Print the position
        print('Latitude: {0:.6f}'.format(gps.latitude), 'Longitude: {0:.6f}'.format(gps.longitude))
    else:
        # print("no gps fix")
        pass
    acc = mpu.acceleration
    acc = np.array([[acc[0]],
                    [acc[1]],
                    [acc[2]]])
    gravity = alpha * gravity + (1 - alpha) * acc

    filtered_acceleration = acc - gravity
    speed += filtered_acceleration * 0.01
    disposition += speed * 0.01

    print_acc = np.round(filtered_acceleration, decimals=2)
    print_speed = np.round(speed, decimals=2)
    print_pos = np.round(disposition, decimals=2)
    print_gyro = np.round(np.array(mpu.gyro), decimals=2)
    # sys.stdout.write(f'\rdX: {print_acc[0, 0]}, dY: {print_acc[1, 0]}, dZ: {print_acc[2, 0]} m/s^2 X: {print_speed[0,0]}, Y: {print_speed[1,0]}, Z: {print_speed[2,0]} m/s')
    sys.stdout.write(f'\rdX: {print_speed[0,0]}, dY: {print_speed[1,0]}, dZ: {print_speed[2,0]} m/s X: {print_pos[0,0]}, Y: {print_pos[1,0]}, Z: {print_pos[2,0]} m oX: {print_gyro[0]}, oY: {print_gyro[1]}, oZ: {print_gyro[2]} rad/s')
    sys.stdout.flush()
    sleep(0.01)
