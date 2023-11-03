import serial
import adafruit_gps
from time import sleep


# Create a serial connection
uart = serial.Serial("/dev/serial0", baudrate=9600)

# Create a GPS module instance
gps = adafruit_gps.GPS(uart, debug=False)

while True:
    # Update GPS data
    gps.update()

    # Check if there are new coordinates available
    if gps.has_fix:
        # Print the position
        print('Latitude: {0:.6f}'.format(gps.latitude), 'Longitude: {0:.6f}'.format(gps.longitude))
    else:
        print("no gps fix")
    sleep(0.1)