import datetime
import logging
import numpy as np
from collections import deque
from time import sleep
import os
import threading
import subprocess
import serial
import RpiMotorLib

GPIO_pins = (-1, -1, -1)  # Microstep Resolution MS1-MS3 -> GPIO Pin
direction = 20  # Direction -> GPIO Pin
step = 21  # Step -> GPIO Pin
mainmotor = RpiMotorLib.A4988Nema(direction, step, GPIO_pins, "A4988")
mainmotor.motor_go(True, "Full", 600, 0.0005, False, 0.0000)
print("motor revolution")
