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

i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)

noise = np.array([0,
                  0,
                  0], dtype=np.float32)

for i in range(40):
    noise += np.float32(0.025) * np.array(mpu.gyro)
    sleep(0.025)



debug_df = []

lasttime = np.float32(perf_counter())
try:
    while True:
        input_gyro = mpu.gyro
        dT = np.float32(perf_counter()) - lasttime
        lasttime = np.float32(perf_counter())
        
        # dRotation = np.array(mpu.gyro, dtype=np.float32) - noise
        # dRotation = np.where(np.less_equal(np.abs(dRotation), np.float32(0.01)), 0, dRotation)
        

        data = {'GyroX': input_gyro[0], 'GyroY': input_gyro[1], 'GyroZ': input_gyro[2],
                'NoiseX': noise[0], 'NoiseY': noise[1], 'NoiseZ': noise[2],
                'dT': dT
                }
        debug_df.append(data)

        # sys.stdout.write(f'\raX: {acceleration[0]:.2f}, aY: {acceleration[1]:.2f}, aZ: {acceleration[2]:.2f} ; vX: {speed[0]:.2f}, vY: {speed[1]:.2f}, zZ: {speed[2]:.2f} ; pX: {position[0]:.2f}, pY: {position[1]:.2f}, pZ: {position[2]:.2f}       ')
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
    filename = f"logs/accelerometerlogs/gyroTest_{current_time}.csv"
    df.to_csv(filename, index=False)

    print(f"Telemetry data saved to {filename}")
