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
import threading
# import pose_move as pm
from pose_moves import PoseMover

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


def pose_recognizer(cv):
       with cv:
            
            image = drone.get_frame_read().frame
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
                cv.notifyAll()

            cv2.putText(image,
                        num_to_pose[pose],
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 2)

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
                drone.land()
                time.sleep(3)
                drone.streamoff()
                time.sleep(2)
                cv2.destroyAllWindows()
                # drone.end()

def move(cv):
    with cv:
        cv.wait()
        if pose_to_do == 0:
            print('take_photo')
        elif pose_to_do == 2:
            drone.rotate_counter_clockwise(360)
        elif pose_to_do == 3:
            drone.rotate_clockwise(360)
        elif pose_to_do == 4:
            drone.move_left(50)
        elif pose_to_do == 5:
            drone.move_right(50)
        elif pose_to_do == 6:
            drone.move_up(50)
        elif pose_to_do == 7:
            drone.curve_xyz_speed(0, 0, 0, 0, 50, 0, 20)
        elif pose_to_do == 8:
            drone.curve_xyz_speed(0, 0, 0, 0, -50, 0, 20)
               


if __name__ == '__main__':
    

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

  

    num_to_pose = {
        -2: 'NESSUN UMANO',
        -1: 'STIMA NON ACCURATA',
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


    drone.connect()
    print(drone.get_battery())
    drone.streamon()
    #pose_mover = PoseMover(drone)
   
    pose_to_do = -1
    state = 0
   
        

   
    print('INIT STATE')
    #black_white_init(drone)
    #time.sleep(5)
    drone.takeoff()
    condition = threading.Condition()
    counter_pose = [0 for _ in range(8)]
    cs1 = threading.Thread(name='consumer1', target=pose_recognizer,args=(condition,))
    cs2 = threading.Thread(name='consumer2', target=move, args=(condition,))
    cs1.start()
    time.sleep(2)
    cs2.start()
    time.sleep(2)

    print('PROGRAMMA TERMINATO')
    #drone.end()
