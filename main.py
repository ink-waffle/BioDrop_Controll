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

    def tryInitialising(self):
        if self.isInitialised is True:
            return
        self.getGPSPosition()
        if self.initialX == -1000000 or self.initialY == -1000000:
            return
        candidate = gpsHandler.getGPSPosition()
        state = np.array([[candidate[0]], [candidate[1]], [candidate[0]], [candidate[1]]])
        if np.isclose(candidate[0], 0) and np.isclose(candidate[1], 0):
            return
        else:
            self.isInitialised = True
            self.stateHistory[1] = state
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
        (count, data) = pi.bb_serial_read(RX_PIN)
        if not count:
            return
        buffer = data.decode('utf-8', errors='ignore')
        sentences = buffer.split('\r\n')
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
        if self.tick % 5 != 0:
            self.tick += 1
            return

        candidate = self.getGPSPosition()
        if np.isclose(candidate[0], 0) and np.isclose(candidate[1], 0):
            self.tick += 1
            return
        if np.sqrt(np.square(candidate[0] - self.lastState[0, 0]) + np.square(
                candidate[0] - self.lastState[0, 0])) > 0.03 * (self.tick - self.rng[-1]):
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
        cos = (a[0, 0] * b[0, 0] + a[1, 0] * b[1, 0]) / (len_a * len_b)
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
            pos - self.lastState[:2])) / (self.tick - self.rng[-1])
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
                                                    self.tick - self.rng[-1])
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
        if self.tick_fromLastUPD % 5 != 0:
            self.tick_fromLastUPD += 1
            return
        position = np.array([[self.state[0, 0]],
                             [self.state[1, 0]]])
        velocity = np.array([[self.state[2, 0]],
                             [self.state[3, 0]]])
        self.tick_fromLastUPD += refreshTime
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

    def receiveNewPosition(self, pos0, pos1, timeBetween):
        if not self.updated:
            self.lastPos = pos0 + ((self.lastPosTick / timeBetween) * (pos1 - pos0))
            self.updated = True
            self.lastPosTick = 0

    def motorControlRoutine(self):
        if self.tick % 5 != 0:
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
            print("motor revolution")
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
@app.route("/motor_control", methods=["POST"])
def motor_control():
    global motorRunning
    motorRunning = not motorRunning


realtimeHandler = None
gpsHandler = None
motorHandler = None
motorRunning = False

if __name__ == "__main__":
    # define GPIO pins
    GPIO_pins = (14, 15, 18)  # Microstep Resolution MS1-MS3 -> GPIO Pin
    direction = 20  # Direction -> GPIO Pin
    step = 21  # Step -> GPIO Pin

    mainmotor = RpiMotorLib.A4988Nema(direction, step, GPIO_pins, "A4988")
    pi = pigpio.pi()
    pi.set_mode(RX_PIN, pigpio.INPUT)
    pi.bb_serial_read_open(23, 9600, 8)

    currentTick = 0  # 10^-2 seconds
    realtimeHandler = RealtimePositionHandler()
    motorHandler = MotorControlHandler(realtimeHandler)
    gpsHandler = GPSHandler(realtimeHandler, motorHandler)
    currentTick = 0
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    while True:
        if gpsHandler.isInitialised is True:
            gpsHandler.receiveGPSData_LR()
        realtimeHandler.updatePostion()
        if motorRunning:
            motorHandler.motorControlRoutine()
        if gpsHandler.isInitialised is False and currentTick % 5 == 0:
            if currentTick % 250 == 0:
                print("initialising...")
            gpsHandler.tryInitialising()
        currentTick += 1
        sleep(0.01)
