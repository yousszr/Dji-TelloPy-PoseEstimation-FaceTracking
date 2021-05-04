import sys
import traceback
# import tellopy
import av
import cv2.cv2 as cv2  # for avoidance of pylint error
import numpy as np
import time
import argparse
import logging
from estimator import TfPoseEstimator
from networks import get_graph_path, model_wh
from djitellopy import tello
# import pose_move as pm
from pose_moves import PoseMover
from simple_pid import PID
import matplotlib.pyplot as plt

logger = logging.getLogger('TfPoseEstimator-WebCam')
logger.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
formatter = logging.Formatter('[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s')
ch.setFormatter(formatter)
logger.addHandler(ch)

fps_time = 0



def distance(A, B):
    return float(np.sqrt((B[0] - A[0]) ** 2 + (B[1] - A[1]) ** 2))


def checkBodyPart(body_parts):
    if body_parts is None:
        return False
    # count = 0
    # for i in range(8):  # from 0 to 7
    #     if i not in body_parts:
    #         count += 1
    # return count < 2


# 0-Nose 1-Neck 2-Rshoulder 3-RElbow 4-RWrist 5-LShoulder 6-LElbow  7-LWrist
# #Pose Recognition
def poseRecognizer(human):
    # if not checkBodyPart(human.body_parts):
    #     return -1  # mancano alcune parti
    th = 0.2
    d_4_0 = distance(human.get_part(4).get_part_coordinate(), human.get_part(0).get_part_coordinate())
    d_7_0 = distance(human.get_part(7).get_part_coordinate(), human.get_part(0).get_part_coordinate())

    if d_4_0 < th and d_7_0 < th:
        return 1  # "ENTRAMBE LE MANI SUL CAPO"
    # elif d_4_0 < th:  # and pose != "MANO SINISTRA SULLA BOCCA":
    #     return 1  # "MANO DESTRA SUL CAPO"
    # elif d_7_0 < th:   # and pose != "MANO DESTRA SULLA BOCCA":
    #     return 2  # "MANO SINISTRA SUL CAPO"

    d_5_4 = distance(human.get_part(5).get_part_coordinate(), human.get_part(4).get_part_coordinate())
    if d_5_4 < th and pose != 4:
        return 2  # "MANO DESTRA SULLA SPALLA SINSITRA"
    d_7_2 = distance(human.get_part(7).get_part_coordinate(), human.get_part(2).get_part_coordinate())
    if d_7_2 < th and pose != 3:
        return 3  # "MANO SINISTRA SULLA SPALLA DESTRA"
    else:
        return -2  # pose non riconosciuta


pose = -1

def black_white_init(drone):
    counter = 0
    flag = True
    while counter < 3:
        frame = drone.get_frame_read().frame

        grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        (thresh, blackAndWhiteFrame) = cv2.threshold(grayFrame, 100, 255, cv2.THRESH_BINARY)

        cv2.imshow('video bw', blackAndWhiteFrame)
        mean = np.mean(blackAndWhiteFrame)
        if not flag and mean > 60:
            flag = True
        if flag and mean < 10:
            counter += 1
            flag = False

        print(f'Counter: {counter}\tMean: {mean}')

        if cv2.waitKey(1) == 27:
            break
    cv2.destroyWindow('video bw')



def facetracking(drone, image,pid_throttle,pid_yaw,face_cascade,axis_speed):
        imageft = cv2.resize(image, (320, 240))
        gray = cv2.cvtColor(imageft, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.2, 8)
        h,w,_ = imageft.shape
        target=[h/2,w/2]     
        ref_x = int(w/2)
        ref_y = int(h*0.35)
        tracked=False

        if len(faces) == 0:
            print("No face detected")
            tracked=False
            # cmd_vel.linear.x = 0.0
            # cmd_vel.angular.z = 0.0
            axis_speed = { "yaw":0, "roll":0, "pitch":0, "throttle":0}

        else:
            for (x, y, w, h) in faces:

                cv2.rectangle(imageft, (x, y), (x + w, y + h), (255, 0, 0), 2)
                roi = imageft[y:y + h, x:x + w]

                target[0] = (x + (w / 2))
                target[1] = (y + (h / 2))

                #body_in_prev_frame = True
                
                xoff = int(target[0]-ref_x)
                yoff = int(ref_y-target[1])
                cv2.circle(imageft, (ref_x, ref_y), 15, (250,150,0), 1,cv2.LINE_AA)
                cv2.arrowedLine(imageft, (ref_x, ref_y), (int(target[0]),int(target[1])), (250, 150, 0), 6)
                if(xoff and  yoff): tracked=True
                # The PID controllers calculate the new speeds for yaw and throttle
                axis_speed["yaw"] = int(-pid_yaw(xoff))
                
                last_rotation_is_cw = axis_speed["yaw"] > 0

                axis_speed["throttle"] = int(-pid_throttle(yoff))
                if(axis_speed["throttle"]==0 and axis_speed["yaw"]==0 and axis_speed["pitch"]==0 and axis_speed["roll"]==0): tracked=True
            drone.send_rc_control(axis_speed["roll"],axis_speed["pitch"],axis_speed["throttle"],axis_speed["yaw"])
        
        if(tracked):
             return imageft, True
        else:
             return imageft, False



def main():
    global pose

    fps_time = 0
    parser = argparse.ArgumentParser(description='tf-pose-estimation realtime webcam')
    # parser.add_argument('--camera', type=int, default=0)
    # parser.add_argument('--zoom', type=float, default=1)
    parser.add_argument('--model', type=str, default='mobilenet_thin_432x368',
                        help='cmu_640x480 / cmu_640x360 / mobilenet_thin_432x368')
    parser.add_argument('--show-process', type=bool, default=False,
                        help='for debug purpose, if enabled, speed for inference is dropped.')
    args = parser.parse_args()

    # logger.debug('initialization %s : %s' % (args.model, get_graph_path(args.model)))
    w, h = model_wh(args.model)
    e = TfPoseEstimator(get_graph_path(args.model), target_size=(w, h))
    logger.debug('cam read+')
    # cam = cv2.VideoCapture(args.camera)
    # ret_val, image = cam.read()
    # logger.info('cam image=%dx%d' % (image.shape[1], image.shape[0]))

    # drone = tellopy.Tello()

    num_to_pose = {
        -2:'NESSUNA POSE',
        -1:'NESSUN UMANO',
        1: 'ENTRAMBE LE MANI SUL CAPO',
        2: 'MANO DESTRA SULLA SPALLA SINSITRA',
        3: 'MANO SINISTRA SULLA SPALLA DESTRA',
        4: 'BRACCIO DESTRO DISTESO',
        5: 'BRACCIO SINISTRO DISTESO',
        6: 'ENTRAMBE BRACCIA DISTESE',
        7: 'MANO DESTRA SUL CAPO',
        8: 'MANO SINISTRA SUL CAPO',

        # 5: "",
        # 6: "",
        # 7: "",
        # 8: "",
        # 9: "",
    }
    


    drone = tello.Tello()
    drone.RESPONSE_TIMEOUT = 5

    pid_yaw = PID(0.25,0,0,setpoint=0,output_limits=(-100,100))
    pid_throttle = PID(0.4,0,0,setpoint=0,output_limits=(-80,100))
    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    axis_speed = { "yaw":0, "roll":0, "pitch":0, "throttle":0}


    drone.connect()
    print(drone.get_battery())
    drone.streamon()
    pose_mover = PoseMover(drone)
    counter_pose = [0 for _ in range(8)]
    pose_to_do = -1
    state = 0
    while state != 5:  # fino a che non e' quello finale
        image = drone.get_frame_read().frame

        if state == 0:  # attende segnale per partenza
            print('INIT STATE')
            #black_white_init(drone)
            time.sleep(5)
            drone.takeoff()
            drone.send_rc_control(0,0,20,0)
        
            state = 1  # TODO mettere facetracking
        elif state == 1:  # partito o finito mossa dopo pose, fa facetracking
            image, tracked = facetracking(drone,image,pid_throttle,pid_yaw,face_cascade,axis_speed)
            if(): state = 2
        elif state == 2:  # pose recognition
            print('POSE RECOGNITION')
            humans = e.inference(image)
            # Pose Recognition
            if len(humans) > 0:
                pose = poseRecognizer(humans[0])
            else:
                pose = -1  # non ce nessuno
            if pose != -1 and pose != -2:
                counter_pose[pose] += 1
            image = TfPoseEstimator.draw_humans(image, humans, imgcopy=False)
            print(counter_pose)
            max_counter_pose = max(counter_pose)
            if max_counter_pose >= 10:  ##################### CHECK VALORE
                counter_pose = [0 for _ in range(8)]
                pose_to_do = pose
                if pose_to_do == 1:  # bisogna terminare
                    state = 4
                else:
                    state = 3
            cv2.putText(image,
                        num_to_pose[pose],
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 2)

        elif state == 3:  # eseguire mossa
            print('MOVE')
            pose_mover.move(pose_to_do)
            time.sleep(6)
            state = 1  # TODO mettere facetracking
        elif state == 4:  # termina
            print('END')
            drone.land()
            time.sleep(3)
            drone.streamoff()
            time.sleep(2)
            cv2.destroyAllWindows()
            # drone.end()
            state = 5

        cv2.putText(image,
                    "FPS: %f" % (1.0 / (time.time() - fps_time)),
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0), 2)

        cv2.putText(image,
                    f"STATE: {state}",
                    (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0), 2)

        # SHOW
        cv2.imshow('Drone Cam', image)
        fps_time = time.time()

        if cv2.waitKey(1) == 27:
            # drone.land()
            state = 5

    print('PROGRAMMA TERMINATO')
    drone.end()


if __name__ == '__main__':
    
    main()
