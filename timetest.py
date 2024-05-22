import numpy as np
import serial
import adafruit_gps
from time import sleep
import board
import adafruit_mpu6050
import numpy
import sys
from time import perf_counter
import pandas as pd
import datetime

dT = 0
lasttime = perf_counter()
for _ in range(1000):
    dT += perf_counter() - lasttime
    lasttime = perf_counter()
    sleep(0.001)

print(str(dT / 1000))
df = pd.DataFrame([])

# Get the current time for the filename
current_time = datetime.datetime.now().strftime("%m%d_%H%M%S")

# Export DataFrame to a CSV file with the current time in the filename
filename = f"logs/accelerometerlogs/accTest_{current_time}.csv"
df.to_csv(filename, index=False)