import datetime
import logging
import pigpio
import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib
import numpy as np
from collections import deque
from time import sleep
from flask import Flask, render_template, request, jsonify
import os
import threading
import subprocess
import serial


class GPSHandler:
    def __init__(self, realtimeH, motorH):
        self.running = False
        self.initialX, self.initialY = -1000000, -1000000
        self.stateHistory = dict()
        self.isInitialised = False
        self.tick = 0
        self.realtimeHandler: RealtimePositionHandler = realtimeH
        self.MotorControlHandler: MotorControlHandler = motorH
        self.lastState: np.array = None
        self.prelastState: np.array = None
        self.rng: deque = None
        # define GPIO pins
        self.pi = pigpio.pi()
        self.pi.set_mode(23, pigpio.INPUT)
        self.pi.bb_serial_read_open(23, 9600, 8)
        # self.ser = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=1)

    def tryInitialising(self):
        if self.isInitialised is True:
            return
        candidate = self.getGPSPosition()
        if self.initialX == -1000000 or self.initialY == -1000000:
            return
        state = np.array([[candidate[0]], [candidate[1]], [candidate[0]], [candidate[1]]])
        if np.isclose(candidate[0], 0) and np.isclose(candidate[1], 0):
            return
        else:
            self.isInitialised = True
            self.stateHistory[25] = state
            self.stateHistory[0] = np.array([[0], [0], [0], [0]])
            self.lastState: np.array = state
            self.prelastState: np.array = np.array([[0], [0], [0], [0]])
            self.realtimeHandler.newPositionReceive(self.lastState)
            self.tick = 26
            self.rng = deque([0, 25])

    def getGPSPosition(self):
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

        a = self.initialX
        b = self.initialY
        c = -1000000
        (count, data) = self.pi.bb_serial_read(23)
        if not count:
            return (0, 0)
        buffer = data.decode('utf-8', errors='ignore')
        sentences = buffer.split('\r\n')
        # sentences = list()
        # line = ''
        # if self.ser.in_waiting > 0:
        #     line = self.ser.readline().decode('utf-8', errors='ignore')
        #     sentences.append(line)
        # while line.endswith('\n'):
        #     line = ''
        #     if self.ser.in_waiting > 0:
        #         line = self.ser.readline().decode('utf-8', errors='ignore')
        #         sentences.append(line)
        for sentence in sentences:
            if not sentence.startswith('$GPGGA'):
                continue
            gps_data = parse_gpgga(sentence)
            if not gps_data:
                continue
            if gps_data['longitude']:
                a = gps_data['longitude']
                if self.initialX == -1000000:
                    self.initialX = a
            if gps_data['latitude']:
                b = gps_data['latitude']
                if self.initialY == -1000000:
                    self.initialY = b
            if gps_data['altitude']:
                c = gps_data['altitude']
        disposition = (a - self.initialX) * 111139, (b - self.initialY) * 111139
        if self.realtimeHandler:
            self.realtimeHandler.logDate(
                f'type: {"Raw"}, longitude: {a}, latitude: {b}, altitude: {c}, disposition: {disposition}')
        return disposition

    def receiveGPSData_LR(self):
        accelerationCoefficient = 1
        bufferSize = 5
        if self.tick % 4 != 0:
            self.tick += 1
            return

        candidate = self.getGPSPosition()
        if np.isclose(candidate[0], 0) and np.isclose(candidate[1], 0):
            self.tick += 1
            return
        dif = np.sqrt(np.square(candidate[0] - self.lastState[0, 0]) + np.square(
            candidate[1] - self.lastState[1, 0]))
        if dif > 0.03 * (self.tick - self.rng[-1]) or dif < 0.01:
            self.tick += 1
            return

        self.stateHistory[self.tick] = np.array(
            [[candidate[0]],
             [candidate[1]],
             [0],
             [0]])
        a = np.array([[self.stateHistory[self.tick][0, 0] - self.lastState[0, 0]],
                      [self.stateHistory[self.tick][1, 0] - self.lastState[1, 0]]])
        b = np.array([[self.lastState[2, 0]],
                      [self.lastState[3, 0]]]) * (self.tick - self.rng[-1])

        len_a = np.linalg.norm(a)
        len_b = np.linalg.norm(b)
        cos = (a[0, 0] * b[0, 0] + a[1, 0] * b[1, 0]) / (len_a * len_b) if len_b * len_a != 0 else 1
        speedDiff = len_a / len_b
        additive = 0
        if cos <= 0.6:
            additive += 1
        if speedDiff >= 1.5:
            additive += 1
        if cos > 0.6 and speedDiff <= 0.5:
            additive += 1
        for _ in range(int(additive)):
            if len(self.rng) > 2:
                self.rng.popleft()
        self.rng.append(self.tick)
        if len(self.rng) > bufferSize:
            self.rng.popleft()

        meanX = np.mean([self.stateHistory[i][0, 0] for i in self.rng])
        meanY = np.mean([self.stateHistory[i][1, 0] for i in self.rng])
        meanTime = np.mean(self.rng)
        k_x = np.sum([(self.stateHistory[i][0, 0] - meanX) * (i - meanTime) for i in self.rng]) / np.sum(
            [(i - meanTime) ** 2 for i in self.rng])
        k_y = np.sum([(self.stateHistory[i][1, 0] - meanY) * (i - meanTime) for i in self.rng]) / np.sum(
            [(i - meanTime) ** 2 for i in self.rng])
        b_x = meanX - (k_x * meanTime)
        b_y = meanY - (k_y * meanTime)

        pos_prediction = np.array([[(k_x * self.tick) + b_x], [(k_y * self.tick) + b_y]])
        moveDirection = np.array([[k_x], [k_y]])
        moveDirection = moveDirection / np.linalg.norm(moveDirection)
        pos = pos_prediction

        speed = (moveDirection * np.linalg.norm(
            pos - self.lastState[:2])) / (self.tick - self.rng[-2])
        speed *= accelerationCoefficient

        self.prelastState = self.lastState
        self.lastState = np.array([
            [pos[0, 0]],
            [pos[1, 0]],
            [speed[0, 0]],
            [speed[1, 0]],
        ])
        self.realtimeHandler.newPositionReceive(self.lastState)
        self.MotorControlHandler.receiveNewPosition(self.prelastState[:2], self.lastState[:2],
                                                    self.tick - self.rng[-2])
        self.realtimeHandler.logDate(
            f'type: {"Processed"}, longitude: {0}, latitude: {0}, altitude {0}, disposition: {(pos[0, 0], pos[1, 0])}, velocity: {(speed[0, 0], speed[1, 0])}')
        self.tick += 1


class RealtimePositionHandler:
    def __init__(self):
        self.currentPosition = np.array([[0], [0]])
        self.tick_fromLastUPD = 0
        self.state = np.array([[0], [0], [0], [0]])

        logs_folder = os.path.abspath(os.path.dirname(__file__)) + '/logs'
        os.makedirs(logs_folder, exist_ok=True)
        self.logger = logging.getLogger('my_logger')
        self.logger.setLevel(logging.DEBUG)
        log_file = os.path.join(logs_folder, datetime.datetime.now().strftime('%m-%d_%H-%M-%S') + '.json')
        fileHandler = logging.FileHandler(log_file)
        fileHandler.setLevel(logging.DEBUG)
        fileHandler.setFormatter(logging.Formatter('%(message)s'))
        self.logger.addHandler(fileHandler)

    def logDate(self, msg):
        if self.logger:
            self.logger.info(msg)
        pass

    def getCurrentPosition(self):
        return self.currentPosition

    def newPositionReceive(self, newState: np.array):
        self.tick_fromLastUPD = 0
        self.state = newState

    def updatePostion(self):
        if self.tick_fromLastUPD % 4 != 0:
            self.tick_fromLastUPD += 1
            return
        position = np.array([[self.state[0, 0]],
                             [self.state[1, 0]]])
        velocity = np.array([[self.state[2, 0]],
                             [self.state[3, 0]]])
        self.tick_fromLastUPD += 1
        self.currentPosition = position + (velocity * self.tick_fromLastUPD)


class MotorControlHandler:
    def __init__(self, realtimeH):
        self.constantDropInterval = 500
        self.minDropInterval = 100
        self.lastPos = (0, 0)
        self.tick = 0
        self.updated = False
        self.lastPosTick = 0
        self.realtimeHandler: realtimeHandler = realtimeH
        # GPIO_pins = (4, 27, 22)  # Microstep Resolution MS1-MS3 -> GPIO Pin
        GPIO_pins = (-1, -1, -1)  # Microstep Resolution MS1-MS3 -> GPIO Pin
        direction = 20  # Direction -> GPIO Pin
        step = 21  # Step -> GPIO Pin
        self.mainmotor = RpiMotorLib.A4988Nema(direction, step, GPIO_pins, "A4988")

    def receiveNewPosition(self, pos0, pos1, timeBetween):
        if not self.updated:
            self.lastPos = pos0 + ((self.lastPosTick / timeBetween) * (pos1 - pos0))
            self.updated = True
            self.lastPosTick = 0

    def motorControlRoutine(self):
        if self.tick % 4 != 0:
            self.tick += 1
            return
        currentPosition = realtimeHandler.getCurrentPosition()
        distance = np.linalg.norm(currentPosition - self.lastPos)
        if (distance >= 10 and self.tick - self.lastPosTick >= self.minDropInterval) or (
                self.tick - self.lastPosTick >= self.constantDropInterval):
            self.lastPos = currentPosition
            self.lastPosTick = self.tick
            self.updated = False
            self.realtimeHandler.logDate(
                f'type: {"Motor"}, longitude: {0}, latitude: {0}, altitude: {0}, disposition: {(currentPosition[0, 0], currentPosition[1, 0])}, velocity: {(0, 0)}')
            self.mainmotor.motor_go(True, "Full", 600, 0.0005, False, 0.0000)
        self.tick += 1


# Initialize Flask app
app = Flask(__name__)


# Define home page route and function
@app.route("/")
def index():
    return render_template("index.html")


@app.route('/position')
def get_position():
    global realtimeHandler
    if realtimeHandler:
        pos = realtimeHandler.getCurrentPosition()
        # realtimeHandler.logDate(
        #     f'type: {"Web"}, longitude: {0}, latitude: {0}, disposition: {(pos[0, 0], pos[1, 0])}, velocity: {(0, 0)}')
        return jsonify({'x': int(pos[0, 0]), 'y': int(pos[1, 0])})
    else:
        return jsonify({'x': 0, 'y': 0})


# Define motor control route and function
@app.route("/control", methods=["POST"])
def motor_control():
    if request.form["submit_button"] == "toggleMotor":
        global motorRunning
        motorRunning = not motorRunning
        return "Motor Running = " + str(motorRunning)
    elif request.form["submit_button"] == "shutDown":
        global running
        running = False
        print("Stopping main loop...")
        sleep(0.1)
        # Execute shutdown command using sudo
        try:
            subprocess.run(["sudo", "shutdown", "-h", "now"], check=True)
        except subprocess.CalledProcessError as e:
            return (f"Error shutting down: {e}")
        else:
            return "Shut Down Successfully"
    else:
        return "Invalid request"


# Define motor control route and function
@app.route("/eport", methods=["POST"])
def motor_control():
    if request.form["submit_button"] == "zip":
        # Create an in-memory byte stream (this will hold our zip file)
        memory_file = io.BytesIO()

        with zipfile.ZipFile(memory_file, 'w') as zf:
            # Here we're zipping all files in the 'files' directory
            # Modify as needed to select which files you want to include
            for root, dirs, files in os.walk('logs'):
                for file in files:
                    zf.write(os.path.join(root, file), file)

        memory_file.seek(0)  # Move cursor to the beginning of the file for reading
        return send_file(memory_file, attachment_filename='logs.zip', as_attachment=True)
    else:
        return "Invalid request"


def turnOff():
    global running
    running = False
    print("Stopping main loop...")

    # Execute shutdown command using sudo
    try:
        subprocess.run(["sudo", "shutdown", "-h", "now"], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error shutting down: {e}")


realtimeHandler = None
gpsHandler = None
motorHandler = None
motorRunning = False
running = True


def runFlask():
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)


if __name__ == "__main__":

    currentTick = 0  # 10^-2 seconds
    realtimeHandler = RealtimePositionHandler()
    motorHandler = MotorControlHandler(realtimeHandler)
    gpsHandler = GPSHandler(realtimeHandler, motorHandler)

    flaskThread = threading.Thread(target=runFlask)
    flaskThread.start()

    while running:
        if gpsHandler.isInitialised is True:
            gpsHandler.receiveGPSData_LR()
        realtimeHandler.updatePostion()
        if motorRunning:
            motorHandler.motorControlRoutine()
        if gpsHandler.isInitialised is False and currentTick % 100 == 0:
            gpsHandler.tryInitialising()
        currentTick += 1
        sleep(0.025)
