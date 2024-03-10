#!/usr/bin/python3

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

import board
import adafruit_mpu6050
from time import perf_counter


class AccelerometerHandler:
    def __init__(self):
        self.running = False
        self.i2c = board.I2C()
        self.mpu = adafruit_mpu6050.MPU6050(self.i2c)
        self.noise = np.array([[0], [0], [0]], dtype=np.float32)
        self.gravity_magnitude = np.float32(0)
        self.initialPitch, self.pitch, self.initialRoll, self.roll, self.initialYawn, self.yawn = np.float32(
            0), np.float32(0), np.float32(0), np.float32(0), np.float32(0), np.float32(0)

    def calibrate(self):
        self.noise = np.array([[0],
                          [0],
                          [0]], dtype=np.float32)
        gravity = np.array([[0.0],
                            [0.0],
                            [0.0]])

        for i in range(1000):
            self.noise += np.float32(0.001) * np.array(self.mpu.gyro).reshape((3, 1))
            gravity += np.float32(0.001) * np.array(self.mpu.acceleration).reshape((3, 1))
            sleep(0.001)

        print_gravity = np.round(gravity, 2)
        gravity_magnitude = np.linalg.norm(gravity)
        gravity_normalized = gravity / gravity_magnitude

        self.initialPitch = self.pitch = np.float32(np.arcsin(-gravity_normalized[0, 0]))
        self.initialRoll = self.roll = np.float32(np.arcsin(gravity_normalized[1, 0] / (np.cos(self.pitch))))
        self.initialYawn = self.yawn = np.float32(0)

        print(f'roll: {np.round(self.initialRoll, 2)} pitch: {np.round(self.initialPitch, 2)}')
        print(f'gravity magnitude: {np.round(gravity_magnitude, 2)}')
        print(f'gX: {print_gravity[0, 0]}, gY: {print_gravity[1, 0]}, gZ: {print_gravity[2, 0]};')

    def accelerometerRoutine(self):
        lastTime = np.float32(perf_counter())
        dRotation, dT, gravity = None, None, None
        while True:
            # noise = 0.01 * drotation + 0.99 * noise
            dRotation = np.array(self.mpu.gyro, dtype=np.float32).reshape((3, 1)) - self.noise
            dRotation = np.where(np.less_equal(np.abs(dRotation), np.float32(0.01)), 0, dRotation)
            dRotation = np.array([[1, np.sin(self.roll) * np.tan(self.pitch), np.cos(self.roll) * np.tan(self.pitch)],
                                  [0, np.cos(self.roll), -np.sin(self.roll)],
                                  [0, np.sin(self.roll) / np.cos(self.pitch), np.cos(self.roll) / np.cos(self.pitch)]]) @ dRotation

            dT = np.float32(perf_counter()) - lasttime
            dRotation *= dT
            self.roll += dRotation[0, 0]
            self.pitch += dRotation[1, 0]
            self.yawn += dRotation[2, 0]
            lasttime = np.float32(perf_counter())

            self.roll += np.float32(0.000003) if self.initialRoll > self.roll else np.float32(-0.000003)
            self.pitch += np.float32(0.000003) if self.initialPitch > self.pitch else np.float32(-0.000003)
            self.yawn += np.float32(0.000003) if self.initialYawn > self.yawn else np.float32(-0.000003)

            gravity = self.gravity_magnitude * np.array([[np.cos(self.yawn) * -np.sin(self.pitch)],
                                                    [np.cos(self.yawn) * np.sin(self.roll) * np.cos(self.pitch) + np.sin(
                                                        self.pitch) * np.sin(self.yawn)],
                                                    [np.cos(self.roll) * np.cos(self.pitch)]])
            acceleration = np.array(self.mpu.acceleration).reshape((3, 1)) - gravity
            sleep(0.01)


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

    def logDate(self, type=None, longitude=0, latitude=0, disposition=(0, 0), velocity=(0, 0)):
        new_row = {'type': type, 'longitude': longitude, 'latitude': latitude, 'dispositionX': disposition[0],
                   'dispositionY': disposition[1], 'velocityX': velocity[0], 'velocityY': velocity[1]}
        self.logFrame = self.logFrame._append(new_row, ignore_index=True)

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


class MotorControlHandler:
    def __init__(self, realtimeH, test=False):
        self.constantDropInterval = 3
        self.minDropInterval = 1
        self.lastPos = np.array([[0], [0]])
        self.time = 0
        self.realtimeHandler: realtimeHandler = realtimeH
        GPIO_pins = (-1, -1, -1)  # Microstep Resolution MS1-MS3 -> GPIO Pin
        direction = 20  # Direction -> GPIO Pin
        step = 21  # Step -> GPIO Pin
        self.mainmotor = RpiMotorLib.A4988Nema(direction, step, GPIO_pins, "A4988")
        self.motorRunning = False
        self.test = test
        self.firstDropDone = False

    def toggleMotor(self):
        if self.motorRunning is False:
            self.time = 0
            self.lastPos = realtimeHandler.getCurrentPosition()
            self.firstDropDone = False
            self.motorRunning = True
            threading.Thread(target=self.motorControlRoutine).start()
        else:
            self.motorRunning = False

    def motorControlRoutine(self):
        while self.motorRunning:
            currentPosition = self.realtimeHandler.getCurrentPosition()
            distance = np.linalg.norm(currentPosition - self.lastPos)
            if self.firstDropDone is False or (distance >= 10 and self.time >= self.minDropInterval) or (
                    self.time >= self.constantDropInterval):
                self.time = 0
                self.firstDropDone = True
                if distance > 0:
                    self.lastPos += ((currentPosition - self.lastPos) / distance) * 10
                if not self.test:
                    self.realtimeHandler.logDate(type='Motor',
                                                 disposition=(currentPosition[0, 0], currentPosition[1, 0]))
                else:
                    self.realtimeHandler.logDate(type='Motor',
                                                 disposition=self.gpsHandler.testData.getPosition(noise=False))
                self.mainmotor.motor_go(True, "Full", 800, 0.0008, False, 0.0000)
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


@app.route('/process', methods=['POST'])
def process_form():
    # Access the submitted data
    input_interval = request.form['input_interval']

    motorHandler.constantDropInterval = int(input_interval)
    print("interval set to: " + str(input_interval))
    return redirect(url_for('index'))


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
    motorHandler = MotorControlHandler(realtimeHandler, gpsHandler, test=False)

    flaskThread = threading.Thread(target=runFlask)
    flaskThread.start()
