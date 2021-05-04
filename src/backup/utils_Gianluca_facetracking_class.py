import cv2
import time
from djitellopy import tello
from simple_pid import PID


class FaceTracking:

    def __init__(self, drone: tello.Thread, width, hight):
        self.drone = drone
        self.width = width
        self.hight = hight

        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        self.axis_speed = {"yaw": 0, "roll": 0, "pitch": 0, "throttle": 0}
        self.pid_yaw = PID(0.25, 0, 0, setpoint=0, output_limits=(-100, 100))
        self.pid_throttle = PID(0.4, 0, 0, setpoint=0, output_limits=(-80, 100))
        self.pid_pitch = PID(0.01, 0, 0, setpoint=0, output_limits=(-15, 15))

        self.target = [hight / 2, width / 2, 3000]
        self.ref_x = int(width / 2)
        self.ref_y = int(hight * 0.25)
        self.ref_z = 3000

        self.th = 7

    def tracking(self):
        counter = 0
        while counter < 10:

            image = self.drone.get_frame_read().frame
            image = cv2.resize(image, (self.width, self.hight))
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.2, 8)
            # h, w, _ = image.shape

            if len(faces) == 0:
                print("No face detected")
                self.axis_speed = {"yaw": 0, "roll": 0, "pitch": 0, "throttle": 0}

            else:
                for (x, y, w, h) in faces:
                    cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    roi = image[y:y + h, x:x + w]

                    self.target[0] = (x + (w / 2))
                    self.target[1] = (y + (h / 2))
                    self.target[2] = w * h  # area

                    # body_in_prev_frame = True

                    xoff = int(self.target[0] - self.ref_x)
                    yoff = int(self.ref_y - self.target[1])
                    zoff = int(self.ref_z - self.target[2])
                    cv2.circle(image, (self.ref_x, self.ref_y), 15, (250, 150, 0), 1, cv2.LINE_AA)
                    cv2.arrowedLine(image, (self.ref_x, self.ref_y), (int(self.target[0]), int(self.target[1])),
                                    (250, 150, 0), 6)

                    # The PID controllers calculate the new speeds for yaw and throttle
                    self.axis_speed["yaw"] = int(-self.pid_yaw(xoff))
                    self.axis_speed["throttle"] = int(-self.pid_throttle(yoff))
                    self.axis_speed["pitch"] = int(-self.pid_pitch(zoff))

                self.drone.send_rc_control(self.axis_speed["roll"], self.axis_speed["pitch"],
                                           self.axis_speed["throttle"],
                                           self.axis_speed["yaw"])
                if abs(self.axis_speed['roll']) < self.th and \
                    abs(self.axis_speed['pitch']) < self.th and \
                    abs(self.axis_speed['throttle']) < self.th and \
                    abs(self.axis_speed['yaw']) < self.th:
                    counter += 1
                else:
                    counter = 0
                print(counter)

            cv2.imshow('Frame', image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                drone.land()
                break



drone = tello.Tello()
drone.RESPONSE_TIMEOUT = 10
drone.connect()
print(drone.get_battery())
drone.streamon()
time.sleep(2)
drone.takeoff()
time.sleep(2)
drone.send_rc_control(0, 0, 15, 0)
# drone.send_rc_control(0, 0, 20, 0)
time.sleep(2.2)

width = 320
hight = 240

facetracking = FaceTracking(drone, width, hight)
facetracking.tracking()

drone.land()
time.sleep(3)
drone.streamoff()
time.sleep(3)
drone.end()
