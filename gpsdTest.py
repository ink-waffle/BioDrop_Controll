import gpsd
from time import sleep
# Connect to the local gpsd
gpsd.connect()

while(True):
    packet = gpsd.get_current()
    # Print the position (latitude and longitude)
    print("Latitude: ", packet.lat)
    print("Longitude: ", packet.lon)
    sleep(1)
