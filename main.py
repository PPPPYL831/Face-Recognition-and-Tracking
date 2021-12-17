#!/usr/bin/env opencv-env

from flask import Flask, render_template, Response, request
import cv2, os, time, threading
import numpy as np
import urllib.request
from time import sleep
import RPi.GPIO as GPIO
import socketio, flask_socketio

# motor
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

panPin = 27
tiltPin = 17
GPIO.setup(panPin, GPIO.OUT)
GPIO.setup(tiltPin, GPIO.OUT)

def setServoAngle(servo, angle):
	assert angle >=30 and angle <= 180
	pwm = GPIO.PWM(servo, 50)
	pwm.start(8)
	dutyCycle = angle / 18. + 1
	pwm.ChangeDutyCycle(dutyCycle)
	sleep(0.3)
	pwm.stop()

global panServoAngle
global tiltServoAngle
panServoAngle = 89
tiltServoAngle = 90

# face position
global xPosition, yPosition
# auto
global move
global consecutiveLoss
move, consecutiveLoss = True, 0
xPosition, yPosition = [], []
xP = 160
yP = 120
auto = 2

def changeAuto(x):
    global auto, lock
    with lock:
        auto = x
    return

def servoPosition():
    global panServoAngle, tiltServoAngle
    global xPosition, yPosition
    if len(xPosition) >= 5:
        print(xPosition, yPosition)

    if (sum(xPosition)//5 < 130):
        panServoAngle += 5
        if panServoAngle > 150:
            panServoAngle = 150
        setServoAngle(panPin, panServoAngle)
    elif (sum(xPosition)//5 > 190):
        panServoAngle -= 5
        if panServoAngle < 30:
            panServoAngle = 30
        setServoAngle(panPin, panServoAngle)
    elif (sum(yPosition)//5 > 140):
        tiltServoAngle += 5
        if tiltServoAngle > 150:
            tiltServoAngle = 150
        setServoAngle(tiltPin, tiltServoAngle)
    elif (sum(yPosition)//5 < 100):
        tiltServoAngle -= 5
        if tiltServoAngle < 30:
            tiltServoAngle = 30
        setServoAngle(tiltPin, tiltServoAngle)
    xPosition, yPosition = [], []

# Flask
app = Flask(__name__)
flaskSocketIO = flask_socketio.SocketIO(app)
outputFrame = None
lock = threading.Lock()
autoL = threading.Lock()

# root page
@app.route('/')
def index():
    """Video streaming home page."""
    global auto
    changeAuto(1)
    print("if auto detect: ", auto)
    return render_template('index.html')

# first page
@app.route('/given_angle.html')
def given_angle():
    global auto
    changeAuto(2)
    print("if auto detect: ", auto)
    templateData = {
      'panServoAngle'	: panServoAngle,
      'tiltServoAngle'	: tiltServoAngle
	}
    return render_template('given_angle.html', **templateData)

# second page
@app.route('/specific_angle.html')
def specific_angle():
    global auto
    changeAuto(2)
    return render_template('specific_angle.html')

# third page
@app.route('/in_de_angle.html')
def in_de_angle():
    global auto
    changeAuto(2)
    print("if auto detect: ", auto)
    templateData = {
      'panServoAngle'	: panServoAngle,
      'tiltServoAngle'	: tiltServoAngle
	}
    return render_template('in_de_angle.html', **templateData)

def gen():
    """Video streaming generator function."""
    global outputFrame, lock
    while True:
        # acquire and release lock automatically
        with lock:
            # check if the output frame is available, otherwise skip
            # the iteration of the loop
            if outputFrame is None:
                continue
            # encode the frame in JPEG format
            (flag, encodedImage) = cv2.imencode(".jpg", outputFrame)
            # ensure the frame was successfully encoded
            if not flag:
                continue
        # yield the output frame in the byte format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')

# require video
@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(), mimetype = 'multipart/x-mixed-replace; boundary=frame')

# socket
@flaskSocketIO.on("socket_set")
def socket_set(data):
    global panServoAngle, tiltServoAngle
    print("socket_set: ", data)
    pin, angle = 0, 90
    if data["servo"] == 'PAN': 
        pin = panPin
        panServoAngle = data["angle"]
        angle = panServoAngle
    else: 
        pin = tiltPin
        tiltServoAngle = data["angle"]
        angle = tiltServoAngle
    setServoAngle(servo=pin, angle=angle)
    return


# initiate recognizer
recognizer = cv2.face.LBPHFaceRecognizer_create()
recognizer.read('faceRecognition/trainer/trainer.yml')
cascadePath = "faceRecognition/haarcascade_frontalface_default.xml"
faceCascade = cv2.CascadeClassifier(cascadePath)
font = cv2.FONT_HERSHEY_SIMPLEX
#initiate id counter
id = 0
# names related to ids: example ==> Marcelo: id=1,  etc
names = ['None', 'ShuhanDing', 'YilingPeng']

def recognize_face_from_mjpg_stream():
    global outputFrame, lock, lockMoving
    global xP, yP, xPosition, yPosition
    global panServoAngle, tiltServoAngle
    global move, consecutiveLoss
    # auto
    while True:
        stream = urllib.request.urlopen(
             "http://{}:8080/?action=snapshot".format(SERVER_ADDR))
        imgnp = np.array(bytearray(stream.read()), dtype=np.uint8)
        img = cv2.imdecode(imgnp, -1)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.2,
            minNeighbors=5,
        )
        # cxP, cyP = 160, 120
        for (x, y, w, h) in faces:
            xP = int(x+w/2)
            yP = int(y+h/2)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            id, confidence = recognizer.predict(gray[y:y + h, x:x + w])

            # Check if confidence is less them 100 ==> "0" is perfect match
            if (confidence < 100):
                id = names[id]
                confidence = "  {0}%".format(round(100 - confidence))
            else:
                id = "unknown"
                confidence = "  {0}%".format(round(100 - confidence))
            cv2.putText(img, str(id), (x + 5, y - 5), font, 1, (255, 255, 255),
                        2)
            cv2.putText(img, str(confidence), (x + 5, y + h - 5), font, 1,
                        (255, 255, 0), 1)
        if auto == 1:
            if len(faces) == 0 and (panServoAngle != 90 or tiltServoAngle != 90):
                if consecutiveLoss == 3:
                    move = False
                    panServoAngle, tiltServoAngle = 90, 90
                    xPosition, yPosition = [], []
                    xP, yP = 160, 120
                    setServoAngle(panPin, 90)
                    setServoAngle(tiltPin, 90)
                    consecutiveLoss = 0
                else:
                    consecutiveLoss += 1
            elif len(faces) > 0:
                move = True

        # acquire and release lock automatically
        with lock:
            outputFrame = img.copy()
            if move and auto == 1:
                if len(xPosition) < 5:
                    xPosition.append(xP)
                    yPosition.append(yP)
                else:
                    servoPosition()

# SERVER_ADDR = "10.49.38.17"
# SERVER_ADDR = "192.168.1.144"


if __name__ == '__main__':

    SERVER_ADDR = os.environ.get('IP')
    print("PYTHON GET IP: ", SERVER_ADDR)

    # initiate socketio
    sio = socketio.Client()
    sio.connect("http://{}:5000".format(SERVER_ADDR))

    time.sleep(1)
    t = threading.Thread(target=recognize_face_from_mjpg_stream)
    t.daemon = True
    t.start()

    # app.run(host='0.0.0.0', port =5000, debug=True, threaded=True)
    flaskSocketIO.run(app, debug=True, host=SERVER_ADDR)
    