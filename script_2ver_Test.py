import bisect
import math
import statistics
from time import sleep
# import pigpio
# import RPi.GPIO as GPIO
# from RpiMotorLib import RpiMotorLib
from flask import Flask, render_template, request, jsonify, redirect
import random
random.seed = 69420
import logging
import os
import datetime
import numpy as np
import threading
from collections import deque
import pandas as pd
import plotly.express as px
import plotly.offline as pyo

# define GPIO pins
GPIO_pins = (14, 15, 18)  # Microstep Resolution MS1-MS3 -> GPIO Pin
direction = 20  # Direction -> GPIO Pin
step = 21  # Step -> GPIO Pin
RX_PIN = 23


# mainmotor = RpiMotorLib.A4988Nema(direction, step, GPIO_pins, "A4988")
# pi = pigpio.pi()
# pi.set_mode(RX_PIN, pigpio.INPUT)
# pi.bb_serial_read_open(RX_PIN, 9600, 8)

realtimeHandler = None
gpsHandler = None
motorHandler = None

currentGPS = np.array([0, 0])
dataFrame = pd.DataFrame([], columns=['x','y','t','type'])

def simulateMovement():
    global currentGPS
    route = {
        0: np.array([76.5180968, 43.1106265]),
        10: np.array([76.5180968, 43.1106265]),
        11: np.array([76.5181148, 43.1106265]),
        13: np.array([76.51814179, 43.1106265]),
        22: np.array([76.51895159, 43.1106265]),
        24: np.array([76.51897858, 43.1106265]),
        25: np.array([76.51899657, 43.1106265]),
        27: np.array([76.51899657, 43.1106355]),
        29: np.array([76.51899657, 43.11067149]),
        30: np.array([76.51897858, 43.11067149]),
        31: np.array([76.51895159, 43.11067149]),
        39: np.array([76.51814179, 43.11067149]),
        40: np.array([76.5180968, 43.11067149]),
        110: np.array([76.5180968, 43.11067149]),
    }
    keys = sorted(list(route.keys()))
    timeInAir = 0
    while True:
        ind = bisect.bisect_right(keys, timeInAir) - 1
        currentGPS = route[keys[ind]] + (timeInAir - keys[ind]) / (keys[ind + 1] - keys[ind]) * (
                route[keys[ind + 1]] - route[keys[ind]])
        timeInAir += 0.1
        sleep(0.1)
class GPSHandler:
    def __init__(self):
        self.running = False
        self.initialX, self.initialY = -1000000, -1000000
        self.stateHistory = dict()  # x, y, dx, dy
        self.filter_dt = 1
        self.isInitialised = False
        initialisationThread = threading.Thread(target=self.tryInitialising)
        initialisationThread.start()

    def tryInitialising(self):
        while self.isInitialised is False:
            self.getGPSPosition()
            if self.initialX == -1000000 or self.initialY == -1000000:
                pass
            else:
                self.isInitialised = True
            sleep(self.filter_dt)
        candidate = gpsHandler.getGPSPosition()
        self.state = np.array([[candidate[0]], [candidate[1]], [candidate[0]], [candidate[1]]])
        while candidate[0] == 0 and candidate[1] == 0:
            sleep(self.filter_dt)
            candidate = gpsHandler.getGPSPosition()
            self.state = np.array([[candidate[0]], [candidate[1]], [candidate[0]], [candidate[1]]])

        self.start()

    def start(self):
        if self.running is False and self.isInitialised:
            self.running = True
            gpsPositionFilteringThread = threading.Thread(target=self.receiveGPSData_LR, args=(self.state,))
            gpsPositionFilteringThread.start()

    def stop(self):
        self.running = False

    def getGPSPosition(self, real=False):
        global realtimeHandler
        global currentGPS

        def parse_gpgga(sentence):
            fields = sentence.split(',')
            if len(fields) < 15:
                return None

            data = {
                'time': fields[1],
                'latitude': float(fields[2]) / 100.0 if fields[2] else None,
                'latitude_direction': fields[3],
                'longitude': float(fields[4]) / 100.0 if fields[4] else None,
                'longitude_direction': fields[5],
                'altitude': float(fields[9]) if fields[9] else None,
                'altitude_units': fields[10]
            }
            return data

        a = self.initialX
        b = self.initialY
        # (count, data) = pi.bb_serial_read(RX_PIN)
        # if not count:
        #     return
        # buffer = data.decode('utf-8', errors='ignore')
        # sentences = buffer.split('\r\n')
        sentences = ['$GPGGA'] if random.random() < 1 else list()
        for sentence in sentences:
            if not sentence.startswith('$GPGGA'):
                continue
            # gps_data = parse_gpgga(sentence)
            if real is False:
                gps_data = {'longitude': currentGPS[0] + ((-0.000017995 * random.random()) + 0.0000089977),
                            'latitude': currentGPS[1] + ((-0.000017995 * random.random()) + 0.0000089977)}
            else:
                gps_data = {'longitude': currentGPS[0],
                            'latitude': currentGPS[1]}
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
        disposition = (a - self.initialX) * 111139, (b - self.initialY) * 111139
        if realtimeHandler:
            realtimeHandler.logDate(
                f'type: {"Raw"}, longitude: {a}, latitude: {b}, disposition: {disposition}, velocity: {(0, 0)}')
        return disposition

    def receiveGPSData_LR(self, initialState: np.array):
        accelerationCoefficient = 1
        bufferSize = 5
        self.stateHistory[1] = initialState
        self.stateHistory[0] = np.array([[0], [0], [0], [0]])
        self.stateHistory[-1] = initialState
        self.stateHistory[-2] = np.array([[0], [0], [0], [0]])
        global realtimeHandler
        realtimeHandler.newPositionReceive(self.stateHistory[-1])
        tick = 2
        rng = deque([0, 1])
        while self.running is True:
            sleep(self.filter_dt)
            candidate = self.getGPSPosition()
            dataFrame.loc[len(dataFrame)] = (candidate[0], candidate[1], tick, 'Raw')
            if candidate[0] == 0 and candidate[1] == 0:
                self.stateHistory[tick] = np.array([
                    [realtimeHandler.getCurrentPosition()[0, 0]],
                    [realtimeHandler.getCurrentPosition()[1, 0]],
                    [self.stateHistory[-1][2, 0]],
                    [self.stateHistory[-1][3, 0]],
                ])
                self.stateHistory[-2] = self.stateHistory[-1]
                self.stateHistory[-1] = self.stateHistory[tick]
                tick += 1
                continue
            self.stateHistory[tick] = np.array(
                [[candidate[0]],
                 [candidate[1]],
                 [0],
                 [0]])
            a = np.array([[self.stateHistory[tick][0, 0] - self.stateHistory[-1][0, 0]],
                          [self.stateHistory[tick][1, 0] - self.stateHistory[-1][1, 0]],
                          [1]])
            b = np.array([[self.stateHistory[-1][2, 0]],
                          [self.stateHistory[-1][3, 0]],
                          [1]])
            len_a = np.linalg.norm(a)
            len_b = np.linalg.norm(b)
            cos = (a[0, 0] * b[0, 0] + a[1, 0] * b[1, 0] + a[2, 0] * b[2, 0])/(len_a*len_b)
            speedDiff = len_a/(len_b*self.filter_dt)
            additive = 0
            if cos <= 0.6:
                additive += 1
            if speedDiff >= 1.5:
                additive += 1
            if cos > 0.6 and speedDiff <= 0.5:
                additive += 1
            # if 0 <= cos <= 0.6:
            #     additive = np.ceil(-5 * cos * ((1.5*len_a)/(len_b * self.filter_dt))) + 6
            # elif -0.6 <=cos < 0:
            #     additive = np.ceil(-5 * cos * ((1.5*len_a)/(len_b * self.filter_dt))) + 4
            # else:
            #     additive = 0
            for _ in range(int(additive)):
                if len(rng) > 2:
                    rng.popleft()
            rng.append(tick)
            if len(rng) > bufferSize:
                rng.popleft()

            meanX = np.mean([self.stateHistory[i][0, 0] for i in rng])
            meanY = np.mean([self.stateHistory[i][1, 0] for i in rng])
            meanTime = np.mean(rng)
            k_x = np.sum([(self.stateHistory[i][0, 0] - meanX)*(i-meanTime) for i in rng]) / np.sum(
                [(i - meanTime) ** 2 for i in rng])
            k_y = np.sum([(self.stateHistory[i][1, 0] - meanY)*(i-meanTime) for i in rng]) / np.sum(
                [(i - meanTime) ** 2 for i in rng])
            b_x = meanX - (k_x * meanTime)
            b_y = meanY - (k_y * meanTime)

            pos_prediction = np.array([[(k_x * tick) + b_x], [(k_y * tick) + b_y]])
            moveDirection = np.array([[k_x], [k_y]])/np.linalg.norm([[k_x], [k_y]])
            # position_change = np.linalg.norm(self.stateHistory[tick][:2] - self.stateHistory[-1][:2])
            # pos_interpolation = self.stateHistory[-1][:2] + moveDirection*position_change
            #
            # pos = pos_prediction + ((pos_interpolation - pos_prediction)*0.01)
            pos = pos_prediction

            distDiff = np.linalg.norm(
                pos - self.stateHistory[-1][:2])

            speed = (moveDirection * (
                distDiff if distDiff < 15 else np.linalg.norm(pos - self.stateHistory[tick - 1][:2]))) / self.filter_dt
            speed *= accelerationCoefficient

            self.stateHistory[-2] = self.stateHistory[-1]
            self.stateHistory[-1] = np.array([
                [pos[0, 0]],
                [pos[1, 0]],
                [speed[0, 0]],
                [speed[1, 0]],
            ])
            realtimeHandler.newPositionReceive(self.stateHistory[-1])
            realtimeHandler.logDate(
                f'type: {"Processed"}, longitude: {0}, latitude: {0}, disposition: {(pos[0, 0], pos[1, 0])}, velocity: {(speed[0, 0], speed[1, 0])}')
            dataFrame.loc[len(dataFrame)] = (pos[0, 0], pos[1, 0], tick, 'Processed' if additive <= 0 else 'Turning')
            tick += 1


class Node:
    def __init__(self, point):
        self.point = point
        self.left = None
        self.right = None


class RealtimePositionHandler:
    def __init__(self):
        self.lastAdded: tuple[np.array, float] = None
        self.currentPosition = np.array([[0], [0]])
        self.timeSinceLastUpdate = 0
        self.running = False
        self.state = np.array([[0], [0], [0], [0]])
        self.logger = None

    def start(self):
        if self.running is False:
            self.running = True
            logs_folder = 'logs'
            os.makedirs(logs_folder, exist_ok=True)
            self.logger = logging.getLogger('my_logger')
            self.logger.setLevel(logging.DEBUG)
            log_file = os.path.join(logs_folder, datetime.datetime.now().strftime('%m-%d_%H-%M-%S') + '.json')
            fileHandler = logging.FileHandler(log_file)
            fileHandler.setLevel(logging.DEBUG)
            fileHandler.setFormatter(logging.Formatter('%(message)s'))
            self.logger.addHandler(fileHandler)
            workingThread = threading.Thread(target=self.updatePostion, args=(0.1,))
            workingThread.start()

    def stop(self):
        self.running = False

    def logDate(self, msg):
        if self.running and self.logger:
            self.logger.info(msg)

    def getCurrentPosition(self, lastAddedNode=None):
        if lastAddedNode is not None:
            self.lastAdded = lastAddedNode, self.timeSinceLastUpdate

        return self.currentPosition

    def newPositionReceive(self, newState: np.array):
        global motorHandler
        if self.lastAdded is not None and self.lastAdded[0] is not None:
            motorHandler.updateNode(self.lastAdded[0], self.state[:2] + (
                    (newState[:2] - self.state[:2]) * (self.lastAdded[1] / self.timeSinceLastUpdate)))
            self.lastAdded = None
        self.timeSinceLastUpdate = 0
        self.state = newState

    def updatePostion(self, refreshTime: float):
        global gpsHandler
        while gpsHandler.running is False:
            sleep(0.1)
        tick = 0
        while self.running is True:
            position = np.array([[self.state[0, 0]],
                                 [self.state[1, 0]]])
            velocity = np.array([[self.state[2, 0]],
                                 [self.state[3, 0]]])
            self.timeSinceLastUpdate += refreshTime
            self.currentPosition = position + (velocity * self.timeSinceLastUpdate)
            tick += 1
            sleep(refreshTime)


class MotorControlHandler:
    def __init__(self):
        self.motor_running = False
        self.droppingPointsRoot: Node = None
        self.dimensions = 2
        self.constantDropInterval = 3.0

    def start(self):
        self.motor_running = True
        motorThread = threading.Thread(target=self.motorControlRoutine, args=(0.1,))
        motorThread.start()

    def stop(self):
        self.motor_running = False


    def toggle(self):
        if self.motor_running is False:
            self.motor_running = True
            motorThread = threading.Thread(target=self.motorControlRoutine, args=(0.1,))
            motorThread.start()
            return "Motor Started"
        if self.motor_running is True:
            self.motor_running = False
            return "Motor Stopped"

    def toggleInPlace(self):
        if self.motor_running is False:
            self.motor_running = True
            inPlaceThread = threading.Thread(target=self.inPlaceRoutine)
            inPlaceThread.start()
            return "In-Place Started"
        if self.motor_running is True:
            self.motor_running = False
            return "In-Place Stopped"

    def removePoint(self, root: Node, point: np.array, depth):
        def areSame(point1, point2):
            return np.allclose(point1, point2)

        if not root:
            return
        if point[depth % self.dimensions, 0] >= root.point[depth % self.dimensions, 0]:
            if root.right and areSame(point, root.right.point) is False:
                self.removePoint(root.right, point, depth + 1)
            else:
                root.right = None
        else:
            if root.left and areSame(point, root.left.point) is False:
                self.removePoint(root.left, point, depth + 1)
            else:
                root.left = None

        return root

    # n - number of dimensions
    def insertPoint(self, root: Node, point: np.array, depth):
        def areSame(point1, point2):
            return np.allclose(point1, point2)

        if not root:
            return Node(point)
        if areSame(root.point, point) is True:
            return root
        if point[depth % self.dimensions, 0] >= root.point[depth % self.dimensions, 0]:
            root.right = self.insertPoint(root.right, point, depth + 1)
        else:
            root.left = self.insertPoint(root.left, point, depth + 1)

        return root

    # n - number of dimensions
    def FindNeighbour(self, root: Node, target):
        currentBestPoint = None
        currentBestDistance = math.inf

        def getSQdist(point1, point2):
            dist = 0
            for i in range(0, len(point1)):
                dist = dist + ((point2[i, 0] - point1[i, 0]) ** 2)
            return dist

        def recurDown(root: Node, target, depth):
            nonlocal currentBestPoint
            nonlocal currentBestDistance
            if not root:
                return math.inf
            c_dm = depth % self.dimensions
            if (target[c_dm] >= root.point[c_dm]):
                recurDown(root.right, target, depth + 1)
            else:
                recurDown(root.left, target, depth + 1)

            dist = getSQdist(root.point, target)
            if dist < currentBestDistance:
                currentBestPoint = root.point
                currentBestDistance = dist

            # check if there can possibly be better options on the other side of the dividing line
            if ((root.point[c_dm] - target[c_dm]) ** 2) > currentBestDistance:
                return
            # Do the same thing for the sibling branch
            if (target[c_dm] >= root.point[c_dm]):
                recurDown(root.left, target, depth + 1)
            else:
                recurDown(root.right, target, depth + 1)

        recurDown(root, target, 0)
        return currentBestPoint, currentBestDistance

    def updateNode(self, node: np.array, newValue: np.array):
        if self.motor_running:
            print("update - from " + str(node.flatten()) + " to " + str(newValue.flatten()))
            self.removePoint(self.droppingPointsRoot, node, 0)
            self.insertPoint(self.droppingPointsRoot, newValue, 0)

    def motorControlRoutine(self, refreshTime: float):
        global realtimeHandler
        global gpsHandler
        if not self.motor_running:
            return
        self.droppingPointsRoot = Node(None)
        pos = realtimeHandler.getCurrentPosition()
        while pos[0, 0] == 0 or pos[1, 0] == 0:
            sleep(0.1)
        self.droppingPointsRoot.point = pos
        # mainmotor.motor_go(True, "Full", 600, 0.0005, False, 0.0000)
        tick = 1
        while self.motor_running:
            sleep(refreshTime)
            currentPosition = realtimeHandler.getCurrentPosition()
            neighbour, distance = self.FindNeighbour(self.droppingPointsRoot, currentPosition)
            if distance >= 25:
                self.insertPoint(self.droppingPointsRoot, currentPosition, 0)
                realtimeHandler.getCurrentPosition(lastAddedNode=currentPosition)
                realtimeHandler.logDate(
                    f'type: {"Motor"}, longitude: {0}, latitude: {0}, disposition: {(currentPosition[0, 0], currentPosition[1, 0])}, velocity: {(0, 0)}')
                # mainmotor.motor_go(True, "Full", 600, 0.0005, False, 0.0000)
                realPosition = gpsHandler.getGPSPosition(real=True)
                dataFrame.loc[len(dataFrame)] = (realPosition[0], realPosition[1], tick, 'Motor')
            tick += 1

    def inPlaceRoutine(self):
        if not self.motor_running:
            return
        while self.motor_running:
            realPosition = gpsHandler.getGPSPosition(real=True)
            dataFrame.loc[len(dataFrame)] = (realPosition[0], realPosition[1], random.randint(0,100000), 'Motor')
            sleep(self.constantDropInterval)


# Initialize Flask app
app = Flask(__name__)


@app.route("/", methods=["GET"])
def index():
    global motorHandler
    input_interval = motorHandler.constantDropInterval
    return render_template('index.html', input_interval=input_interval)


@app.route('/position')
def get_position():
    global realtimeHandler
    if realtimeHandler:
        pos = realtimeHandler.getCurrentPosition()
        realtimeHandler.logDate(
            f'type: {"Web"}, longitude: {0}, latitude: {0}, altitude: {0}, disposition: {(pos[0, 0], pos[1, 0])}, velocity: {(0, 0)}')
        return jsonify({'x': int(pos[0, 0]), 'y': int(pos[1, 0])})
    else:
        return jsonify({'x': 0, 'y': 0})


@app.route('/process', methods=["POST"])
def process():
    global motorHandler
    input_interval = request.form['input_interval']
    motorHandler.constantDropInterval = float(input_interval)
    #     return redirect(url_for('index', input_interval=motorHandler.constantDropInterval))
    print('Drop Interval Set to: ', motorHandler.constantDropInterval)
    return redirect('/?input_interval={}'.format(input_interval))


# Define motor control route and function
@app.route("/motor_control", methods=["POST"])
def motor_control():
    global motorHandler
    if request.form["submit_button"] == "toggleMotor":
        return motorHandler.toggle()
    elif request.form["submit_button"] == "inPlace":
        pyo.plot(px.scatter(dataFrame, x='x', y='y', animation_group='t', color='type', text='t'),
                 filename="scatter_plot.html", auto_open=True)
        return motorHandler.toggleInPlace()
    else:
        return "Invalid request"



if __name__ == "__main__":
    try:
        simulationThread = threading.Thread(target=simulateMovement)
        simulationThread.start()

        gpsHandler = GPSHandler()
        realtimeHandler = RealtimePositionHandler()
        realtimeHandler.start()
        motorHandler = MotorControlHandler()
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        # GPIO.cleanup()
        # pi.set_mode(RX_PIN, pigpio.INPUT)
        # pi.bb_serial_read_close(RX_PIN)
        # pi.stop()
        pass
