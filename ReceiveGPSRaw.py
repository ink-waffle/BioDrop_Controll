import serial
import os
from time import sleep
import logging
import datetime
import threading
# Configure the serial connection
serial_port = 'COM5'  # Update with your COM port
baud_rate = 9600

# Open the serial connection
ser = serial.Serial(serial_port, baud_rate)

logs_folder = 'raw_logs'
os.makedirs(logs_folder, exist_ok=True)
logger = logging.getLogger('my_logger')
logger.setLevel(logging.DEBUG)
log_file = os.path.join(logs_folder, datetime.datetime.now().strftime('%m-%d_%H-%M-%S') + '.json')
fileHandler = logging.FileHandler(log_file)
fileHandler.setLevel(logging.DEBUG)
fileHandler.setFormatter(logging.Formatter('%(message)s'))
logger.addHandler(fileHandler)

def getGPSPosition():
    def parse_gpgga(sentence):
        fields = sentence.split(',')
        if len(fields) < 15:
            return None

        data = {
            'time': fields[1],
            'latitude': int(float(fields[2]) // 100) + ((float(fields[2]) % 100) / 60.0) if fields[2] else None,
            'latitude_direction': fields[3],
            'longitude': int(float(fields[4]) // 100) + ((float(fields[4]) % 100) / 60.0) if fields[4] else None,
            'longitude_direction': fields[5],
            'altitude': float(fields[9]) if fields[9] else None,
            'altitude_units': fields[10]
        }
        return data

    a = -1000000
    b = -1000000
    c = -1000000
    while True:
        line = ser.readline()
        line = line.decode('utf-8', errors='ignore').strip()
        if not line.startswith('$GPGGA'):
            continue
        try:
            gps_data = parse_gpgga(line)
        except:
            return
        if not gps_data:
            return
        if gps_data['longitude']:
            a = gps_data['longitude']
        if gps_data['latitude']:
            b = gps_data['latitude']
        if gps_data['altitude']:
            c = gps_data['altitude']
        logger.info(
                f'type: {"Raw"}, longitude: {a}, latitude: {b}, altitude: {c}')
        return
def readPosition():
    while True:
        getGPSPosition()
        sleep(0.25)


workingThread = threading.Thread(target=readPosition)
workingThread.start()