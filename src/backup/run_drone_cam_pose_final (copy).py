import sys
import traceback
import av
import cv2.cv2 as cv2  # for avoidance of pylint error
import numpy as np
import time
import argparse
import logging
from estimator import TfPoseEstimator
from networks import get_graph_path, model_wh
from djitellopy import tello
from pose_moves_final import PoseMover
from simple_pid import PID
import matplotlib.pyplot as plt

logger = logging.getLogger('TfPoseEstimator-Drone')
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

# 0-Nose 1-Neck 2-Rshoulder 3-RElbow 4-RWrist 5-LShoulder 6-LElbow  7-LWrist
# #Pose Recognition
def poseRecognizer(human):
    # --- CONDIZIONI PER LE RICONOSCIMENTO DELLE POSE ---

    # condizioni nelle variabili, non so se è meglio, è più leggibile, sarebbe da provare con il drone :)

    # mano e gomito all'altezza della spalla circa
    range_braccio = np.float64(0.1) # si potrebbe diminuire leggermente
    mano_dx_y = human.get_part(4).get_part_coordinate()[1]
    gomito_dx_y = human.get_part(3).get_part_coordinate()[1]
    spalla_dx_y = human.get_part(2).get_part_coordinate()[1]

    braccio_dx_disteso = (spalla_dx_y - range_braccio) <= gomito_dx_y <= (spalla_dx_y + range_braccio) and \
            (spalla_dx_y - range_braccio) <= mano_dx_y <= (spalla_dx_y + range_braccio)

    mano_sx_y = human.get_part(7).get_part_coordinate()[1]
    gomito_sx_y = human.get_part(6).get_part_coordinate()[1]
    spalla_sx_y = human.get_part(5).get_part_coordinate()[1]

    braccio_sx_disteso = (spalla_sx_y - range_braccio) <= gomito_sx_y <= (spalla_sx_y + range_braccio) and \
            (spalla_sx_y - range_braccio) <= mano_sx_y <= (spalla_sx_y + range_braccio)

    # entrambi mani e gomiti all'altezza delle spalle
    braccia_distese = braccio_dx_disteso and braccio_sx_disteso

    # mano all'altezza delle orecchie
    orcchie_y = human.get_part(16).get_part_coordinate()[1]
    range_mano = np.float64(0.1)    # si può variare un po'

    mano_dx_alzata = (orcchie_y + range_mano) >= mano_dx_y >= (orcchie_y - range_mano)
    mano_sx_alzata = (orcchie_y + range_mano) >= mano_sx_y >= (orcchie_y - range_mano)

    # entrambe le mani all'altezza delle orecchie
    mani_alzate = mano_dx_alzata and mano_sx_alzata



    target=human.get_part(0).get_part_coordinate()
    d_4_0 = distance(human.get_part(4).get_part_coordinate(), human.get_part(0).get_part_coordinate())
    d_2_5 = distance(human.get_part(2).get_part_coordinate(), human.get_part(5).get_part_coordinate())*960 #Per adattarlo alla misura del frame
    target=(target[0]*960,target[1]*720,d_2_5)
    th = 0.2
    d_4_0 = distance(human.get_part(4).get_part_coordinate(), human.get_part(0).get_part_coordinate())
    d_7_0 = distance(human.get_part(7).get_part_coordinate(), human.get_part(0).get_part_coordinate())
    if d_4_0 < th and d_7_0 < th:
        return 1,target  # "ENTRAMBE LE MANI SUL CAPO"
    d_5_4 = distance(human.get_part(5).get_part_coordinate(), human.get_part(4).get_part_coordinate())
    if d_5_4 < th :
        return 2,target  # "MANO DESTRA SULLA SPALLA SINSITRA"
    d_7_2 = distance(human.get_part(7).get_part_coordinate(), human.get_part(2).get_part_coordinate())
    if d_7_2 < th :
        return 3,target  # "MANO SINISTRA SULLA SPALLA DESTRA"
    if mani_alzate:
        return 6, target
    elif mano_dx_alzata:
        return 5, target
    elif mano_sx_alzata:
        return 4, target
    else:
        return -2,target



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



def main():
    global pose

    fps_time = 0
    parser = argparse.ArgumentParser(description='tf-pose-estimation realtime webcam')
    parser.add_argument('--model', type=str, default='mobilenet_thin_432x368',
                        help='cmu_640x480 / cmu_640x360 / mobilenet_thin_432x368')
    parser.add_argument('--show-process', type=bool, default=False,
                        help='for debug purpose, if enabled, speed for inference is dropped.')
    args = parser.parse_args()
    w, h = model_wh(args.model)
    e = TfPoseEstimator(get_graph_path(args.model), target_size=(w, h))
    logger.debug('cam read+')

    num_to_pose = {
        -2:'NESSUNA POSE',
        -1:'NESSUN UMANO',
        1: 'ENTRAMBE LE MANI SUL CAPO',
        2: 'MANO DESTRA SULLA SPALLA SINSITRA',
        3: 'MANO SINISTRA SULLA SPALLA DESTRA',
        4: 'MANO DESTRA ALZATA',
        5: 'MANO SINSITRA ALZATA',
        6: 'MANI IN ALTO',
    
    }
    
    drone = tello.Tello()
    drone.RESPONSE_TIMEOUT = 5
    axis_speed = { "yaw":0, "roll":0, "pitch":0, "throttle":0, "flip_left":0,"flip_right":0}
    states= { 0:"Avvio", 1:"POSE RECOGNITION E FACE TRACKING",2:"RICERCA...", 4:"SPEGNIMENTO", 6:"PROGRAMMA TERMINATO"}
    messages= { 6:"FOTO SCATTATA :)", 4:"FLIP LEFT ESEGUITO",5:"FLIP RIGHT ESEGUITO"}

    last_yaw=0
    drone.connect()
    print(drone.get_battery())
    drone.streamon()
    pose_mover = PoseMover(drone)
    pose_to_do = -1
    state = 0
    target=(960/2,720*0.35,0)



    while state != 5:  # fino a che non e' quello finale
        image = drone.get_frame_read().frame
        if state == 0:  # attende segnale per partenza
            print('INIT STATE')
            #black_white_init(drone)
            #time.sleep(5)
            #drone.takeoff()
            state = 1  
        elif state == 1:  
            print('POSE RECOGNITION E FACE TRACKING')
            humans = e.inference(image)
            # Pose Recognition
            if len(humans) > 0:
                pose,target = poseRecognizer(humans[0])
            else:
                state=2
                pose = -1  # non ce nessuno 
            image = TfPoseEstimator.draw_humans(image, humans, imgcopy=False)
            pose_to_do = pose
            if pose_to_do == 1:  # bisogna terminare
                    state = 1
            elif pose_to_do == -1  :
                    axis_speed = { "yaw":0, "roll":0, "pitch":0, "throttle":0, "flip_left":0,"flip_right":0}
            else :
                 axis_speed,image = pose_mover.move(pose_to_do,axis_speed,target[0],target[1],target[2],image)

            cv2.putText(image,
                        num_to_pose[pose],
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 255, 0), 2)
            
            #Mando i comandi decisi dal pose_move e il face trakcer
            if(axis_speed["flip_left"]==6):     
                drone.flip_left()
                humans=[]
            if(axis_speed["flip_right"]==6):     
                drone.flip_right()
                humans=[]
            else:
                drone.send_rc_control(axis_speed["roll"],axis_speed["pitch"],axis_speed["throttle"],axis_speed["yaw"])
            last_yaw=axis_speed["yaw"]
        
        elif state == 2:
             print('RICERCA HUMAN')
             if last_yaw!=0:
               drone.send_rc_control(0,0,0,last_yaw)
             else :
               drone.send_rc_control(0,0,0,20)
             humans = e.inference(image)
             if len(humans) > 0:
                 drone.send_rc_control(0,0,0,0)
                 state=1
        
            
        elif state == 4:  # termina
            print('END')
            drone.land()
            state = 6

        elif state == 6:
            state=6
            


        cv2.putText(image,
                    "FPS: %f" % (1.0 / (time.time() - fps_time)),
                    (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 255, 0), 2)

        cv2.putText(image,
                    f"STATE: {states[state]}",
                    (10,150), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 255, 0), 2)

        # SHOW
        cv2.imshow('Drone Cam', image)
        fps_time = time.time()

        if cv2.waitKey(1) == 27:
            # drone.land()
            state = 5

        if cv2.waitKey(1) == 32:
            drone.emergency()
            

    print('PROGRAMMA TERMINATO')
    drone.end()


if __name__ == '__main__':
    
    main()
