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
from adafruit_extended_bus import ExtendedI2C as I2C

uart = serial.Serial("/dev/serial0", baudrate=9600)
i2c_1 = I2C(1)
i2c_2 = I2C(3)
mpu_1 = adafruit_mpu6050.MPU6050(i2c_1)
mpu_2 = adafruit_mpu6050.MPU6050(i2c_2)
# mpu.sleep = False
# mpu.cycle = True
# mpu.cycle_Rate = adafruit_mpu6050.Rate.CYCLE_40_HZ
# mpu.gyro_range = adafruit_mpu6050.GyroRange.RANGE_500_DPS
# mpu.filter_bandwidth = adafruit_mpu6050.Bandwidth.BAND_184_HZ
# sleep(0.1)
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

for _ in range(100):
    input_gyro_1 = mpu_1.gyro
    input_gyro_2 = mpu_2.gyro
    input_gyro = np.array([(input_gyro_1[0] - input_gyro_2[0])/2,
                           (input_gyro_1[1] - input_gyro_2[1])/2,
                           (input_gyro_1[2] + input_gyro_2[2])/2], dtype=np.float32)
    input_acc_1 = mpu_1.acceleration
    input_acc_2 = mpu_2.acceleration
    input_acc = np.array([(input_acc_1[0] - input_acc_2[0])/2,
                           (input_acc_1[1] - input_acc_2[1])/2,
                           (input_acc_1[2] + input_acc_2[2])/2], dtype=np.float32)
    noise += np.float32(0.01) * input_gyro
    gravity += np.float32(0.01) * input_acc
    sleep(0.01)


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
        input_gyro_1 = mpu_1.gyro
        input_gyro_2 = mpu_2.gyro
        input_gyro = np.array([(input_gyro_1[0] - input_gyro_2[0])/2,
                            (input_gyro_1[1] - input_gyro_2[1])/2,
                            (input_gyro_1[2] + input_gyro_2[2])/2], dtype=np.float32)
        input_acc_1 = mpu_1.acceleration
        input_acc_2 = mpu_2.acceleration
        input_acc = np.array([(input_acc_1[0] - input_acc_2[0])/2,
                            (input_acc_1[1] - input_acc_2[1])/2,
                            (input_acc_1[2] + input_acc_2[2])/2], dtype=np.float32)
        dT = np.float32(perf_counter()) - lasttime
        lasttime = np.float32(perf_counter())
        
        dRotation = input_gyro - noise
        dRotation = np.where(np.less_equal(np.abs(dRotation), np.float32(0.05)), 0, dRotation)
        
        # gravity_normalized_mid = gravity_normalized + (np.cross(-dRotation, gravity_normalized) * dT / 2)
        # ox_mid = ox + (np.cross(-dRotation, ox) * dT / 2)
        # oy_mid = oy + (np.cross(-dRotation, oy) * dT / 2)
        gravity_normalized_mid = gravity_normalized
        ox_mid = ox
        oy_mid = oy

        gravity_normalized += np.cross(-dRotation, gravity_normalized_mid) * dT
        ox += np.cross(-dRotation, ox_mid) * dT
        oy += np.cross(-dRotation, oy_mid) * dT

        # gravity_normalized /= np.linalg.norm(gravity_normalized)
        ox /= np.linalg.norm(ox)
        oy /= np.linalg.norm(oy)
        # gravity_normalized /= np.linalg.norm(gravity_normalized)

        gravity = gravity_normalized * gravity_magnitude
        # acceleration = np.array([np.dot(acceleration, ox),
        #                          np.dot(acceleration, oy),
        #                          np.dot(acceleration, gravity_normalized)])
        acceleration = input_acc - gravity
        acceleration_noise = 0.995 * acceleration_noise + 0.005 * acceleration
        acceleration_denoised = acceleration
        acceleration_denoised = np.where(np.less_equal(np.abs(acceleration_denoised), np.float32(0.1)), 0, acceleration_denoised)
        speed += acceleration_denoised * dT
        # speed -= np.float32(0.001) * speed / np.linalg.norm(speed)
        position += np.where(np.less_equal(np.abs(speed), np.float32(0.2)), 0, speed) * dT

        data = {'InpGyro_1X': input_gyro_1[0], 'InpGyro_1Y': input_gyro_1[1], 'InpGyro_1Z': input_gyro_1[2],
                'InpGyro_2X': input_gyro_2[0], 'InpGyro_2Y': input_gyro_2[1], 'InpGyro_2Z': input_gyro_2[2],
                'GyroX': dRotation[0], 'GyroY': dRotation[1], 'GyroZ': dRotation[2],
                'ExpGravityX': gravity_normalized[0], 'ExpGravityY': gravity_normalized[1],  'ExpGravityZ': gravity_normalized[2],
                'ExpOxX': ox[0], 'ExpOxY': ox[1], 'ExpOxZ': ox[2],
                'ExpOyX': oy[0], 'ExpOyY': oy[1], 'ExpOyZ': oy[2],
                'AccX': acceleration[0], 'AccY': acceleration[1], 'AccZ': acceleration[2],
                'AccNoiseX': acceleration_noise[0], 'AccNoiseY': acceleration_noise[1], 'AccNoiseZ': acceleration_noise[2],
                'ExpSpeedX': speed[0], 'ExpSpeedY': speed[1], 'ExpSpeedZ': speed[2],
                'dT': dT
                }
        debug_df.append(data)

        sys.stdout.write(f'\rgX: {gravity[0]:.2f}, gY: {gravity[1]:.2f}, gZ: {gravity[2]:.2f} ; aX: {input_acc[0]:.2f}, aY: {input_acc[1]:.2f}, aZ: {input_acc[2]:.2f}     ')
        # sys.stdout.write(f'\raX: {acceleration_denoised[0]:.2f}, aY: {acceleration_denoised[1]:.2f}, aZ: {acceleration_denoised[2]:.2f} ; vX: {speed[0]:.2f}, vY: {speed[1]:.2f}, vZ: {speed[2]:.2f} ; pX: {position[0]:.2f}, pY: {position[1]:.2f}, pZ: {position[2]:.2f}       ')
        # sys.stdout.write(f'\raX: {acceleration_denoised[0]:.2f}, aY: {acceleration_denoised[1]:.2f}, aZ: {acceleration_denoised[2]:.2f} ; vX: {speed[0]:.2f}, vY: {speed[1]:.2f}, vZ: {speed[2]:.2f}     ')
        sys.stdout.flush()
        sleep(0.01)

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
