import numpy as np
import serial
import adafruit_gps
from time import sleep
import board
import adafruit_mpu6050
import numpy
import sys
from time import perf_counter

dT = 0
lasttime = perf_counter()
for _ in range(1000):
    dT += perf_counter() - lasttime
    lasttime = perf_counter()
    sleep(0.001)

print(dT / 1000)