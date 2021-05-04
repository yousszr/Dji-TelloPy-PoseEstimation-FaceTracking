import cv2
import numpy as np
import time
from djitellopy import tello
from simple_pid import PID

import matplotlib.pyplot as plt


def main():
    drone = tello.Tello()
    drone.connect()
    print(drone.get_battery())
    drone.streamon()
    time.sleep(2)
    drone.takeoff()
    time.sleep(2)
    # drone.send_rc_control(0, 0, 25, 0)
    #drone.send_rc_control(0, 0, 20, 0)
    time.sleep(2.2)

    width = 320
    height = 240

    i = 0
    video_writer = cv2.VideoWriter('output.avi', -1, 20.0, (320,240))

    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    axis_speed = { "yaw":0, "roll":0, "pitch":0, "throttle":0}
    pid_yaw = PID(0.25,0,0,setpoint=0,output_limits=(-100,100))
    pid_throttle = PID(0.4,0,0,setpoint=0,output_limits=(-80,100))

    while True:

        image = drone.get_frame_read().frame
        image = cv2.resize(image, (width, height))
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.2, 8)
        h,w,_ = image.shape
        target=[h/2,w/2]     
        ref_x = int(w/2)
        ref_y = int(h*0.35)

        if len(faces) == 0:
            print("No face detected")
            # cmd_vel.linear.x = 0.0
            # cmd_vel.angular.z = 0.0
            axis_speed = { "yaw":0, "roll":0, "pitch":0, "throttle":0}

        else:
            for (x, y, w, h) in faces:

                cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                roi = image[y:y + h, x:x + w]

                target[0] = (x + (w / 2))
                target[1] = (y + (h / 2))

                #body_in_prev_frame = True
                
                xoff = int(target[0]-ref_x)
                yoff = int(ref_y-target[1])
                cv2.circle(image, (ref_x, ref_y), 15, (250,150,0), 1,cv2.LINE_AA)
                cv2.arrowedLine(image, (ref_x, ref_y), (int(target[0]),int(target[1])), (250, 150, 0), 6)
                       
                # The PID controllers calculate the new speeds for yaw and throttle
                axis_speed["yaw"] = int(-pid_yaw(xoff))
                
                last_rotation_is_cw = axis_speed["yaw"] > 0

                axis_speed["throttle"] = int(-pid_throttle(yoff))
                
            drone.send_rc_control(axis_speed["roll"],axis_speed["pitch"],axis_speed["throttle"],axis_speed["yaw"])
                
        video_writer.write(image)
        cv2.imshow('Frame', image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            drone.land()
            break
    
    drone.streamoff()
    drone.end()
    capture.release()
    video_writer.release()
    cv2.destroyAllWindows()
    





if __name__ == '__main__':
    main()
