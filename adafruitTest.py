import datetime
import serial
import adafruit_gps
from time import sleep, perf_counter
import board
import sys
import adafruit_mpu6050
import numpy as np
import sys
import pandas as pd

uart = serial.Serial("/dev/serial0", baudrate=9600)
i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)
mpu.gyro_range = adafruit_mpu6050.GyroRange.RANGE_500_DPS
gps = adafruit_gps.GPS(uart, debug=False)

noise = np.array([0,
                  0,
                  0], dtype=np.float32)
acceleration_noise = np.array([0,
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

for i in range(100):
    noise += np.float32(0.01) * np.array(mpu.gyro)
    gravity += np.float32(0.01) * np.array(mpu.acceleration)
    sleep(0.01)


# Infer Rotation
print_gravity = np.round(gravity, 2)
gravity_magnitude = np.linalg.norm(gravity)
gravity_normalized = gravity / gravity_magnitude
ox = np.array([1, 0, 0], dtype=np.float32)
ox -= np.dot(ox, gravity_normalized) * gravity_normalized
ox /= np.linalg.norm(ox)
oy = np.cross(ox, gravity_normalized)

debug_df = []

lasttime = np.float32(perf_counter())
try:
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
        input_gyro = mpu.gyro
        input_acc = np.array(mpu.acceleration)
        dT = np.float32(perf_counter()) - lasttime
        lasttime = np.float32(perf_counter())
        
        dRotation = np.array(mpu.gyro, dtype=np.float32) - noise
        dRotation = np.where(np.less_equal(np.abs(dRotation), np.float32(0.01)), 0, dRotation)
        
        gravity_normalized_mid = gravity_normalized + (np.cross(-dRotation, gravity_normalized) * dT / 2)
        ox_mid = ox + (np.cross(-dRotation, ox) * dT / 2)
        oy_mid = oy + (np.cross(-dRotation, oy) * dT / 2)
        # gravity_normalized_mid = gravity_normalized
        # ox_mid = ox
        # oy_mid = oy

        gravity_normalized += np.cross(-dRotation, gravity_normalized_mid) * dT
        ox += np.cross(-dRotation, ox_mid) * dT
        oy += np.cross(-dRotation, oy_mid) * dT

        # gravity_normalized /= np.linalg.norm(gravity_normalized)
        ox /= np.linalg.norm(ox)
        oy /= np.linalg.norm(oy)

        gravity = (gravity_normalized / np.linalg.norm(gravity_normalized)) * gravity_magnitude
        # acceleration = np.array([np.dot(acceleration, ox),
        #                          np.dot(acceleration, oy),
        #                          np.dot(acceleration, gravity_normalized)])
        acceleration = np.where(np.less_equal(np.abs(input_acc), np.float32(0.2)), 0, input_acc)
        speed += acceleration * dT
        # speed -= np.float32(0.001) * speed / np.linalg.norm(speed)
        position += np.where(np.less_equal(np.abs(speed), np.float32(0.2)), 0, speed) * dT

        data = {'GyroX': input_gyro[0], 'GyroY': input_gyro[1], 'GyroZ': input_gyro[2],
                'NoiseX': noise[0], 'NoiseY': noise[1], 'NoiseZ': noise[2],
                'ExpGravityX': gravity_normalized[0], 'ExpGravityY': gravity_normalized[1],  'ExpGravityZ': gravity_normalized[2],
                'ExpOxX': ox[0], 'ExpOxY': ox[1], 'ExpOxZ': ox[2],
                'ExpOyX': oy[0], 'ExpOyY': oy[1], 'ExpOyZ': oy[2],
                'AccX': input_acc[0], 'AccY': input_acc[1], 'AccZ': input_acc[2],
                'ExpSpeedX': speed[0], 'ExpSpeedY': speed[1], 'ExpSpeedZ': speed[2],
                'dT': dT
                }
        debug_df.append(data)

        lasttime = np.float32(perf_counter())
        sys.stdout.write(f'\rgX: {gravity[0]:.2f}, gY: {gravity[1]:.2f}, gZ: {gravity[2]:.2f} ; aX: {acceleration[0]:.2f}, aY: {acceleration[1]:.2f}, aZ: {acceleration[2]:.2f}     ')
        # sys.stdout.write(f'\raX: {acceleration[0]:.2f}, aY: {acceleration[1]:.2f}, aZ: {acceleration[2]:.2f} ; vX: {speed[0]:.2f}, vY: {speed[1]:.2f}, zZ: {speed[2]:.2f} ; pX: {position[0]:.2f}, pY: {position[1]:.2f}, pZ: {position[2]:.2f}       ')
        sys.stdout.flush()
        sleep(0.1)

except KeyboardInterrupt:
    # Graceful exit on interrupt
    print("KeyboardInterrupt detected. Saving data and exiting...")
    
finally:
    # Convert list of telemetry data to a pandas DataFrame
    df = pd.DataFrame(debug_df)

    # Get the current time for the filename
    current_time = datetime.datetime.now().strftime("%m%d_%H%M%S")

    # Export DataFrame to a CSV file with the current time in the filename
    filename = f"logs/accelerometerlogs/accTest_{current_time}.csv"
    df.to_csv(filename, index=False)

    print(f"Telemetry data saved to {filename}")
