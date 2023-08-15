import numpy as np
from collections import deque
from time import sleep


class GPSHandler:
    def __init__(self, realtimeH, motorH):
        self.running = False
        self.initialX, self.initialY = -1000000, -1000000
        self.stateHistory = dict()
        self.isInitialised = False
        self.tick = 0
        self.accumulatedTicks = 1
        self.realtimeHandler: realtimeHandler = realtimeH
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
            self.tick = 2
            self.accumulatedTicks = 1
            self.rng = deque([0, 1])

    def getGPSPosition(self):
        a = self.initialX
        b = self.initialY
        # Here a and b should be received from gps
        disposition = (a - self.initialX) * 111139, (b - self.initialY) * 111139
        # if self.realtimeHandler:
        #     self.realtimeHandler.logDate(
        #         f'type: {"Raw"}, longitude: {a}, latitude: {b}, disposition: {disposition}')
        return disposition

    def receiveGPSData_LR(self, filter_dt: float):
        accelerationCoefficient = 1
        bufferSize = 5
        candidate = self.getGPSPosition()
        if np.isclose(candidate[0], 0) and np.isclose(candidate[1], 0):
            self.accumulatedTicks += 1
            return
        if np.sqrt(np.square(candidate[0] - self.lastState[0, 0]) + np.square(candidate[0] - self.lastState[0, 0])) > 3 * filter_dt * self.accumulatedTicks:
            self.accumulatedTicks += 1
            return
        self.stateHistory[self.tick] = np.array(
            [[candidate[0]],
             [candidate[1]],
             [0],
             [0]])
        a = np.array([[self.stateHistory[self.tick][0, 0] - self.lastState[0, 0]],
                      [self.stateHistory[self.tick][1, 0] - self.lastState[1, 0]]])
        b = np.array([[self.lastState[2, 0]],
                      [self.lastState[3, 0]]]) * self.accumulatedTicks * filter_dt

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
            pos - self.lastState[:2])) / (filter_dt * self.accumulatedTicks)
        speed *= accelerationCoefficient

        self.prelastState= self.lastState
        self.lastState = np.array([
            [pos[0, 0]],
            [pos[1, 0]],
            [speed[0, 0]],
            [speed[1, 0]],
        ])
        self.realtimeHandler.newPositionReceive(self.lastState)
        self.MotorControlHandler.receiveNewPosition(self.prelastState[:2], self.lastState[:2], filter_dt * self.accumulatedTicks)
        # realtimeHandler.logDate(
        #     f'type: {"Processed"}, longitude: {0}, latitude: {0}, altitude {0}, disposition: {(pos[0, 0], pos[1, 0])}, velocity: {(speed[0, 0], speed[1, 0])}')
        self.tick += 1
        self.accumulatedTicks = 1



class RealtimePositionHandler:
    def __init__(self):
        self.currentPosition = np.array([[0], [0]])
        self.timeSinceLastUpdate = 0
        self.running = False
        self.state = np.array([[0], [0], [0], [0]])

        # logs_folder = 'logs'
        # os.makedirs(logs_folder, exist_ok=True)
        # self.logger = logging.getLogger('my_logger')
        # self.logger.setLevel(logging.DEBUG)
        # log_file = os.path.join(logs_folder, datetime.datetime.now().strftime('%m-%d_%H-%M-%S') + '.json')
        # fileHandler = logging.FileHandler(log_file)
        # fileHandler.setLevel(logging.DEBUG)
        # fileHandler.setFormatter(logging.Formatter('%(message)s'))
        # self.logger.addHandler(fileHandler)

    def logDate(self, msg):
        # if self.running and self.logger:
        #     self.logger.info(msg)
        pass

    def getCurrentPosition(self):
        return self.currentPosition

    def newPositionReceive(self, newState: np.array):
        self.timeSinceLastUpdate = 0
        self.state = newState

    def updatePostion(self, refreshTime: float):
        position = np.array([[self.state[0, 0]],
                             [self.state[1, 0]]])
        velocity = np.array([[self.state[2, 0]],
                             [self.state[3, 0]]])
        self.timeSinceLastUpdate += refreshTime
        self.currentPosition = position + (velocity * self.timeSinceLastUpdate)


class MotorControlHandler:
    def __init__(self):
        self.constantDropInterval = 3.0
        self.lastPos = (0, 0)
        self.tick = 0
        self.updated = False
        self.lastPosTick = 0
    def receiveNewPosition(self, pos0, pos1, timeBetween):
        if not self.updated:
            self.lastPos = pos0 + ((self.lastPosTick / timeBetween) * (pos1 - pos0))
            self.updated = True
            self.lastPosTick = 0
    def motorControlRoutine(self, refreshTime):
        currentPosition = realtimeHandler.getCurrentPosition()
        distance = np.linalg.norm(currentPosition - self.lastPos)
        if distance >= 10:
            self.lastPos = currentPosition
            self.lastPosTick = self.tick
            self.updated = False
            # realtimeHandler.logDate(
            #     f'type: {"Motor"}, longitude: {0}, latitude: {0}, altitude: {0}, disposition: {(currentPosition[0, 0], currentPosition[1, 0])}, velocity: {(0, 0)}')
            print("motor revolution")
        self.tick += refreshTime



if __name__ == "__main__":
    while True:
        currentTick = 0# 10^-2 seconds
        realtimeHandler = RealtimePositionHandler()
        motorHandler = MotorControlHandler()
        gpsHandler = GPSHandler(realtimeHandler, motorHandler)
        while gpsHandler.isInitialised is False:
            gpsHandler.tryInitialising()
            sleep(0.25)
        currentTick = 0
        while True:
            if currentTick % 25 == 0:
                gpsHandler.receiveGPSData_LR(0.25)
            if currentTick % 5 == 0:
                realtimeHandler.updatePostion(0.05)
                motorHandler.motorControlRoutine(0.05)
            currentTick += 5
            sleep(0.05)