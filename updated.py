import datetime
import logging
from RpiMotorLib import RpiMotorLib
import numpy as np
from collections import deque
from time import sleep
import time
from flask import Flask, render_template, request, jsonify, send_file, redirect, url_for
import os
import threading
import subprocess
import serial
import adafruit_gps
import pandas as pd
import io
import zipfile
import random

class TestSimulator:
    def lerp(self, start, end, fraction):
        """Linearly interpolate between start and end based on fraction."""
        return (start[0] + fraction * (end[0] - start[0]),
                start[1] + fraction * (end[1] - start[1]))

    def __init__(self):
        """Initialize the GPS simulator with a dictionary of timestamps and positions."""
        self.data = \
            {
                0: (0, 0),
                30: (0, 150),  # Moving up at 5 m/s for 150 meters = 30 seconds
                31: (5, 150),  # Taking 1 second for 90-degree turn
                51: (105, 150),  # Moving right at 5 m/s for 100 meters = 20 seconds
                52: (105, 145),  # Taking 1 second for 90-degree turn
                68: (105, 65),  # Moving down at 5 m/s for 80 meters = 16 seconds
                69: (112, 65),  # U-turn right side 7 meters = 1 second
                70: (112, 58),  # U-turn bottom side 7 meters = 1 second
                71: (105, 58),  # U-turn left side 7 meters = 1 second
                97: (105, 188),  # Moving up at 5 m/s for 130 meters = 26 seconds
                98: (100, 188),  # Taking 1 second for 90-degree turn
                116: (0, 188),  # Moving left at 5 m/s for 100 meters = 18 seconds
                117: (0, 179),  # U-turn top side 9 meters = 1 second
                118: (-9, 179),  # U-turn right side 9 meters = 1 second
                119: (-9, 170),  # U-turn bottom side 9 meters = 1 second
                139: (111, 170),  # Moving right at 5 m/s for 120 meters = 20 seconds
                169: (111, 370),  # Moving up at 5 m/s for 200 meters = 30 seconds
                170: (106, 370),  # Taking 1 second for 90-degree turn
                # Rest of the data based on remaining distance
            }
        self.keys = sorted(self.data.keys())
        self.start_time = time.time()

    def getPosition(self, noise=True):
        """Return the interpolated position based on the current timestamp."""
        if noise and random.random() <= 0.05:
            return (0, 0)
        current_time = time.time() - self.start_time
        for i in range(len(self.data) - 1):
            t_start, pos_start = self.keys[i], self.data[self.keys[i]]
            t_end, pos_end = self.keys[i + 1], self.data[self.keys[i + 1]]

            if t_start <= current_time < t_end:
                fraction = (current_time - t_start) / (t_end - t_start)
                pos = self.lerp(pos_start, pos_end, fraction)
                if noise:
                    return pos[0] + (2 * random.random() - 1), pos[1] + (2 * random.random() - 1)
                else:
                    return pos

        # If current time is after the last timestamp, just return the last position
        return self.data[-1][1]


class GPSHandler:
    def __init__(self, realtimeH, test=False):
        self.running = False
        self.initialX, self.initialY = -1000000, -1000000
        self.stateHistory = [0] * 2
        self.realtimeHandler: RealtimePositionHandler = realtimeH
        # self.MotorControlHandler: MotorControlHandler = motorH
        self.lastState: np.array = None
        self.prelastState: np.array = None
        self.isTest = False

        threading.Thread(target=self.tryInitialising, args=(test,)).start()

    def tryInitialising(self, test=False):
        if self.running is True:
            return
        if test:
            self.isTest = test
            self.testData = TestSimulator()
        if not test:
            uart = serial.Serial("/dev/serial0", baudrate=9600)
            self.gps = adafruit_gps.GPS(uart, debug=False)

        self.running = True
        candidate: np.array = self.getGPSPosition()
        while np.allclose(candidate, 0):
            sleep(0.25)
            candidate: np.array = self.getGPSPosition()

        state = np.vstack((candidate, candidate / 0.25))

        self.stateHistory[1] = state
        self.stateHistory[0] = np.array([[0], [0], [0], [0]])
        self.lastState: np.array = state
        self.prelastState: np.array = np.array([[0], [0], [0], [0]])
        self.realtimeHandler.newPositionReceive(self.lastState)
        sleep(0.25)
        threading.Thread(target=self.receiveGPSData_LR).start()

    def getGPSPosition(self):
        if self.isTest:
            pos = self.testData.getPosition()
            if self.realtimeHandler:
                self.realtimeHandler.logDate(type="Raw", disposition=pos)
            return np.array([[pos[0]], [pos[1]]])
        self.gps.update()
        if self.gps.has_fix:
            if self.initialX == -1000000 or self.initialY == -1000000:
                self.initialX, self.initialY = self.gps.latitude, self.gps.longitude
                return np.array([[0], [0]])
            a, b = self.gps.latitude, self.gps.longitude
            disposition = (a - self.initialX) * 111139, (b - self.initialY) * 111139
            if self.realtimeHandler:
                self.realtimeHandler.logDate(type="Raw", longitude=a, latitude=b, disposition=disposition)
            return np.array([[disposition[0]], [disposition[1]]])
        else:
            return np.array([[0], [0]])

    def receiveGPSData_LR(self, t_diff=1):
        tDiff = t_diff
        while True:
            candidate = self.getGPSPosition()
            if np.allclose(candidate, 0):
                sleep(0.25)
                # self.receiveGPSData_LR(t_diff + 1)
                tDiff += 1
                continue

            self.stateHistory.append(np.vstack((candidate, [0], [0])))

            self.prelastState = self.lastState
            self.lastState = 0.75 * self.lastState + 0.25 * self.stateHistory[-1]
            self.lastState[2:4] = (self.lastState - self.prelastState)[0:2] / (0.25 * t_diff)

            self.realtimeHandler.newPositionReceive(self.lastState)
            # self.MotorControlHandler.receiveNewPosition(self.prelastState[:2], self.lastState[:2], )
            self.realtimeHandler.logDate(type="Processed", disposition=(self.lastState[0, 0], self.lastState[1, 0]), velocity=(self.lastState[2, 0], self.lastState[3, 0]))
            sleep(0.25)
            tDiff = 1


class RealtimePositionHandler:
    def __init__(self):
        self.currentPosition = np.array([[0], [0]])
        self.lastUPD_time = time.time()
        self.state = np.array([[0], [0], [0], [0]])
        self.running = False
        self.logFrame = pd.DataFrame(
            columns=['type', 'longitude', 'latitude', 'dispositionX', 'dispositionY', 'velocityX', 'velocityY'])

    def initialize(self):
        if self.running:
            return
        self.running = True
        # logs_folder = os.path.abspath(os.path.dirname(__file__)) + '/logs'
        # os.makedirs(logs_folder, exist_ok=True)
        # self.logger = logging.getLogger('my_logger')
        # self.logger.setLevel(logging.DEBUG)
        # log_file = os.path.join(logs_folder, datetime.datetime.now().strftime('%m-%d_%H-%M-%S') + '.json')
        # fileHandler = logging.FileHandler(log_file)
        # fileHandler.setLevel(logging.DEBUG)
        # fileHandler.setFormatter(logging.Formatter('%(message)s'))
        # self.logger.addHandler(fileHandler)

        # threading.Thread(target=self.updatePostion).start()

    def logDate(self, type=None, longitude=0, latitude=0, disposition=(0, 0), velocity=(0, 0)):
        self.logFrame.loc[len(self.logFrame)] = {'type': type,
                                                  'longitude': longitude, 'latitude': latitude,
                                                  'dispositionX': disposition[0], 'dispositionY': disposition[1],
                                                  'velocityX': velocity[0], 'velocityY': velocity[1]}


    def saveLogs(self):
        try:
            logs_folder = os.path.join(os.path.abspath(os.getcwd()), 'logs')
            os.makedirs(logs_folder, exist_ok=True)
            self.logFrame.to_csv(os.path.join(logs_folder, datetime.datetime.now().strftime('%m-%d_%H-%M-%S') + '.csv'))
        except Exception as e:
            print(f"An error occurred saving logs: {e}")

    def getCurrentPosition(self):
        position = np.array([[self.state[0, 0]],
                             [self.state[1, 0]]])
        velocity = np.array([[self.state[2, 0]],
                             [self.state[3, 0]]])

        self.currentPosition = position + (velocity * (time.time() - self.lastUPD_time))
        return self.currentPosition

    def newPositionReceive(self, newState: np.array):
        self.lastUPD_time = time.time()
        self.state = newState

    # def updatePostion(self):
    #     while True:
    #         position = np.array([[self.state[0, 0]],
    #                              [self.state[1, 0]]])
    #         velocity = np.array([[self.state[2, 0]],
    #                              [self.state[3, 0]]])
    #
    #         self.currentPosition = position + (velocity * self.time_fromLastUPD)
    #         self.time_fromLastUPD += 0.05
    #         sleep(0.05)


class MotorControlHandler:
    def __init__(self, realtimeH, gpsH, test=False):
        self.constantDropInterval = 10
        self.minDropInterval = 1
        self.lastPos = np.array([[0], [0]])
        self.time = 0
        self.realtimeHandler: realtimeHandler = realtimeH
        self.gpsHandler: gpsHandler = gpsH
        GPIO_pins = (-1, -1, -1)  # Microstep Resolution MS1-MS3 -> GPIO Pin
        direction = 20  # Direction -> GPIO Pin
        step = 21  # Step -> GPIO Pin
        self.mainmotor = RpiMotorLib.A4988Nema(direction, step, GPIO_pins, "A4988")
        self.motorRunning = False
        self.test = test
    def toggleMotor(self):
        if self.motorRunning is False:
            self.time = 0
            self.lastPos = np.array([[0.0], [0.0]])
            self.motorRunning = True
            threading.Thread(target=self.motorControlRoutine).start()
        else:
            self.motorRunning = False

    def motorControlRoutine(self):
        while self.motorRunning:
            currentPosition = realtimeHandler.getCurrentPosition()
            distance = np.linalg.norm(currentPosition - self.lastPos)
            if np.allclose(self.lastPos, 0) or (distance >= 10 and self.time >= self.minDropInterval) or (
                    self.time >= self.constantDropInterval):
                self.time = 0
                if distance > 0:
                    self.lastPos += ((currentPosition - self.lastPos) / distance) * 10
                if not self.test:
                    self.realtimeHandler.logDate(type='Motor', disposition=(currentPosition[0, 0], currentPosition[1, 0]))
                else:
                    self.realtimeHandler.logDate(type='Motor',
                                                 disposition=self.gpsHandler.testData.getPosition(noise=False))
                self.mainmotor.motor_go(True, "Full", 600, 0.0005, False, 0.0000)
                print("motor revolution")
            sleep(0.1)
            self.time += 0.1


log = logging.getLogger('werkzeug')
log.setLevel(logging.WARNING)
app = Flask(__name__)


@app.route("/")
def index():
    return render_template("index.html")


@app.route('/position')
def get_position():
    global realtimeHandler
    if realtimeHandler:
        pos = realtimeHandler.getCurrentPosition()
        return jsonify({'x': int(pos[0, 0]), 'y': int(pos[1, 0])})
    else:
        return jsonify({'x': 0, 'y': 0})


@app.route("/control", methods=["POST"])
def motor_control():
    if request.form["submit_button"] == "toggleMotor":
        motorHandler.toggleMotor()
        return redirect(url_for('index'))
    elif request.form["submit_button"] == "shutDown":
        realtimeHandler.saveLogs()
        print("Stopping main loop...")
        sleep(0.1)
        try:
            subprocess.run(["sudo", "shutdown", "-h", "now"], check=True)
        except subprocess.CalledProcessError as e:
            return (f"Error shutting down: {e}")
        else:
            return "Shut Down Successfully"
    else:
        return "Invalid request"


@app.route("/export", methods=["POST"])
def export():
    if request.form["submit_button"] == "zip":
        # Convert DataFrame to CSV and XML
        csv_data = realtimeHandler.logFrame.to_csv(index=False)
        xml_data = realtimeHandler.logFrame.to_xml(index=False, parser='etree')

        # Create a zip file in memory
        mem_zip = io.BytesIO()

        with zipfile.ZipFile(mem_zip, mode='w', compression=zipfile.ZIP_DEFLATED) as zipf:
            # Write CSV file to zip
            zipf.writestr('data.csv', csv_data)

            # Write XML file to zip
            zipf.writestr('data.xml', xml_data)

        # Set pointer to the beginning of the memory file
        mem_zip.seek(0)

        # Send the zip file for download
        return send_file(mem_zip, download_name='data.zip', as_attachment=True)
    else:
        return "Invalid request"



realtimeHandler = None
gpsHandler = None
motorHandler = None


def runFlask():
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)


if __name__ == "__main__":
    realtimeHandler = RealtimePositionHandler()
    realtimeHandler.initialize()
    gpsHandler = GPSHandler(realtimeHandler, test=False)
    motorHandler = MotorControlHandler(realtimeHandler, gpsHandler, test=False)


    flaskThread = threading.Thread(target=runFlask)
    flaskThread.start()
