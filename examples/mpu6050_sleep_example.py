# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_mpu6050
from time import sleep, perf_counter
import numpy as np

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
mpu = adafruit_mpu6050.MPU6050(i2c)
# mpu.clock_source = adafruit_mpu6050.ClockSource.CLKSEL_INTERNAL_8MHz
# This example is meant to be used with the serial plotter which makes
# it easier to see how the readings change with different settings.
# Make sure to poke and prod the sensor while the demo is running to
# generate some interesting data!
lasttime = perf_counter()
lastacc = np.array([0, 0, 0], dtype=np.float32)
time = np.float32(0)
for _ in range(1000):
    acc = np.array(mpu.acceleration, dtype=np.float32)
    if np.array_equal(acc, lastacc):
        print(f"Same Acceleration detected: {acc} and {lastacc}")
        break
    lastacc = acc
    time += perf_counter() - lasttime
    lasttime = perf_counter()
    
print(f"Average time was: {time/1000}")
# while True:
#     # first show some 'normal' readings
#     print('Normal:\n')
#     mpu.reset()
#     time.sleep(0.1)
#     for count in range(0, 40):
#         print(mpu.acceleration)
#         time.sleep(0.1)

#     print('Cycle:\n')
#     # Next, set a slow cycle rate so the effect can be seen clearly.
#     mpu.cycle_Rate = adafruit_mpu6050.Rate.CYCLE_5_HZ
#     # ensure that we're not sleeping or cycle won't work
#     mpu.sleep = False
#     # Finally, enable cycle mode
#     mpu.cycle = True

#     for count in range(0, 40):
#         print(mpu.acceleration)
#         time.sleep(0.1)

#     print('Sleep:\n')
#     # Finally enable sleep mode. Note that while we can still fetch
#     #  data from the measurement registers, the measurements are not
#     #  updated. In sleep mode the accelerometer and gyroscope are
#     #  deactivated to save power, so measurements are halted.

#     mpu.cycle = False
#     mpu.sleep = True

#     for count in range(0, 40):
#         print(mpu.acceleration)
#         time.sleep(0.1)