import board
import busio
import adafruit_gps

# Create a serial connection
uart = busio.UART(board.TX, board.RX, baudrate=9600)

# Create a GPS module instance
gps = adafruit_gps.GPS(uart)

# Update GPS data
gps.update()

# Print the position
print('Latitude: {0:.6f}'.format(gps.latitude))
print('Longitude: {0:.6f}'.format(gps.longitude))
