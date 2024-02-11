import numpy as np
import serial
import adafruit_gps
from time import sleep
import board
import adafruit_mpu6050
import numpy
# Create a serial connection
uart = serial.Serial("/dev/serial0", baudrate=9600)
i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)
# Create a GPS module instance
gps = adafruit_gps.GPS(uart, debug=False)
gravity = np.array([[0],
                    [0],
                    [0]])
while True:
    # Update GPS data
    gps.update()

    # Check if there are new coordinates available
    if gps.has_fix:
        # Print the position
        print('Latitude: {0:.6f}'.format(gps.latitude), 'Longitude: {0:.6f}'.format(gps.longitude))
    else:
        print("no gps fix")
    # print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (mpu.acceleration))
    alpha = 0.8
    acc = mpu.acceleration
    acc = np.array([[acc[0]],
                    [acc[1]],
                    [acc[2]]])
    gravity = alpha * gravity + (1 - alpha) * acc

    linear_acceleration = acc - gravity
    print('X: ' + str(linear_acceleration[0, 0]) + ' Y: ' + str(linear_acceleration[1, 0]) + ' Z: ' + str(linear_acceleration[2, 0]))
    sleep(0.05)
