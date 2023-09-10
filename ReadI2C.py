import random

import smbus

import time
import board

import adafruit_pcf8591.pcf8591 as PCF
from adafruit_pcf8591.analog_in import AnalogIn
from adafruit_pcf8591.analog_out import AnalogOut

address = 0x48
A0 = 0x40
A1 = 0x41
A2 = 0x42
A3 = 0x43

bus = smbus.SMBus(1)

i2c = board.I2C()
pcf = PCF.PCF8591(i2c)

pcf_in_0 = AnalogIn(pcf, PCF.A0)
pcf_in_1 = AnalogIn(pcf, PCF.A1)
pcf_in_2 = AnalogIn(pcf, PCF.A2)
pcf_in_3 = AnalogIn(pcf, PCF.A3)
pcf_out = AnalogOut(pcf, PCF.OUT)
while True:

    # bus.write_byte(address, A0)
    # value = bus.read_byte(address)
    # # value = int(((value/255 * 5) - 1.65) * 3000)
    # x = int(((value / 255) * 6000) - 3000)
    # time.sleep(0.033)
    #
    # bus.write_byte(address, A1)
    # value = bus.read_byte(address)
    # y = int(((value / 255) * 6000) - 3000)
    # time.sleep(0.033)
    #
    # bus.write_byte(address, A2)
    # value = bus.read_byte(address)
    # z = int(((value / 255) * 6000) - 3000)
    # print("X Y Z : " + str(x) + " " + str(y) + " " + str(z) + " mg     ", end='\r')

    a_0 = pcf_in_0.value
    time.sleep(0.1)

    a_1 = pcf_in_1.value
    time.sleep(0.1)

    a_2 = pcf_in_2.value
    time.sleep(0.1)

    a_3 = pcf_in_3.value
    print("A0 A1 A2 A3 : " + str(a_0) + " " + str(a_1) + " " + str(a_2) + " " + str(a_3) + "    ", end='\r')
    time.sleep(0.1)
    # while True:
    #     a = random.randint(0, 65525)
    #     print("Setting out to ", a)
    #     pcf_out.value = a
    #     raw_value = pcf_in_0.value
    #     scaled_value = (raw_value / 65535) * pcf_in_0.reference_voltage
    #
    #     print("Pin 0: %0.2fV" % (scaled_value))
    #     print("")
    #     time.sleep(1)
