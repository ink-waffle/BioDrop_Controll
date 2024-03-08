import serial
import adafruit_gps
from time import sleep
import board
import sys
import adafruit_mpu6050
import numpy as np

# Create a serial connection
uart = serial.Serial("/dev/serial0", baudrate=9600)
i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)
# Create a GPS module instance
gps = adafruit_gps.GPS(uart, debug=False)

while True:
    # Update GPS data
    gps.update()

    # Check if there are new coordinates available
    if gps.has_fix:
        # Print the position
        print('Latitude: {0:.6f}'.format(gps.latitude), 'Longitude: {0:.6f}'.format(gps.longitude))
    else:
        print("no gps fix")
    print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (mpu.acceleration))
    print("Gyro X:%.2f, Y: %.2f, Z: %.2f rad/s" % (mpu.gyro))
    print_acceleration = np.round(np.array(mpu.acceleration), 2)
    print_gyro = np.round(np.array(mpu.gyro), 2)
    sys.stdout.write(f'\raX: {print_acceleration[0]}, aY: {print_acceleration[1]}, aZ: {print_acceleration[2]} rX: {print_gyro[0]}, rY: {print_gyro[1]}, rZ: {print_gyro[2]}')
    sys.stdout.flush()
    sleep(0.1)
