import smbus

import time

address = 0x48
A0 = 0x40
A1 = 0x41
A2 = 0x42
A3 = 0x43

bus = smbus.SMBus(1)

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
    bus.write_byte(address, A0)
    a_0 = bus.read_byte(address)
    time.sleep(0.1)
    bus.write_byte(address, A1)
    a_1 = bus.read_byte(address)
    time.sleep(0.1)
    bus.write_byte(address, A2)
    a_2 = bus.read_byte(address)
    time.sleep(0.1)
    bus.write_byte(address, A3)
    a_3 = bus.read_byte(address)
    print("A0 A1 A2 A3 : " + str(a0) + " " + str(a1) + " " + str(a2) + " " + str(a3) + "    ", end='\r')
    time.sleep(0.1)


# import time
# import board
# import busio
# import adafruit_adxl34x
#
# address = 0x48
# i2c = busio.I2C(board.SCL, board.SDA)
# accelerometer = adafruit_adxl34x.ADXL345(i2c, address=address)
#
# while True:
#     print("%f %f %f"%accelerometer.acceleration)
#     time.sleep(1)