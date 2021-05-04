import cv2
import numpy as np
from djitellopy import tello
import time


def findFace(faceCascade, img):
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(imgGray, 1.2, 8)
    myFaceListC = []
    myFaceListArea = []
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cx = x + w // 2
        cy = y + h // 2
        area = w * h
        cv2.circle(img, (cx, cy), 5, (0, 255, 0), cv2.FILLED)
        myFaceListC.append([cx, cy])
        myFaceListArea.append(area)
    if len(myFaceListArea) != 0:
        i = myFaceListArea.index(max(myFaceListArea))
        return img, [myFaceListC[i], myFaceListArea[i]]
    else:
        return img, [[0, 0], 0]


def trackFace(info, w, pid, pError):
    x, y = info[0]
    area = info[1]
    fb = 0
    error = x - w // 2
    speed = pid[0] * error + pid[1] * (error - pError)
    speed = int(np.clip(speed, -100, 100))
    if fbRange[0] < area < fbRange[1]:
        fb = 0
    elif area > fbRange[1]:
        fb = -10
    elif area < fbRange[0] and area != 0:
        fb = 10
    if x == 0:
        speed = 0
        error = 0
    drone.send_rc_control(0, fb, 0, speed)
    return error


drone = tello.Tello()
drone.connect()
print(drone.get_battery())
drone.streamon()
drone.takeoff()
# drone.send_rc_control(0, 0, 25, 0)
drone.send_rc_control(0, 0, 20, 0)
time.sleep(2.2)


w, h = 360, 240
fbRange = [6200, 6800]
# fbRange = [5200, 5800]
pid = [0.4, 0.4, 0]
pError = 0
faceCascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")

while True:
    img = drone.get_frame_read().frame
    img = cv2.resize(img, (w, h))
    img, info = findFace(faceCascade, img)
    pError = trackFace(info, w, pid, pError)
    cv2.imshow("Output", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        drone.land()
        break

drone.streamoff()
drone.end()
