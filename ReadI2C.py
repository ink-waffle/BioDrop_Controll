import smbus

import time

address = 0x48
A0 = 0x40
A1 = 0x41
A2 = 0x42

bus = smbus.SMBus(1)

while True:

    bus.write_byte(address, A0)

    value = bus.read_byte(address)

    print(value)

    time.sleep(0.1)

    bus.write_byte(address, A1)

    value = bus.read_byte(address)

    print(value)

    time.sleep(0.1)

    bus.write_byte(address, A2)

    value = bus.read_byte(address)

    print(value)

    time.sleep(0.1)