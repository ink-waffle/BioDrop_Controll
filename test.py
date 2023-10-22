import datetime
import logging
import numpy as np
from collections import deque
from time import sleep
from flask import Flask, render_template, request, jsonify, send_from_directory, Response, send_file
import os
import threading
import subprocess
import serial
import io
import zipfile

# Initialize Flask app
app = Flask(__name__)


# Define home page route and function
@app.route("/")
def index():
    return render_template("index.html")


@app.route('/position')
def get_position():
    return jsonify({'x': 30, 'y': 5})


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
@app.route("/export", methods=["POST"])
def export_logs():
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
        return send_file(memory_file, download_name='logs.zip', as_attachment=True)
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

def runFlask():
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)


if __name__ == "__main__":

    currentTick = 0  # 10^-2 seconds

    flaskThread = threading.Thread(target=runFlask)
    flaskThread.start()
