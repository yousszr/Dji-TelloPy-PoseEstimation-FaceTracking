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
from pose_moves import PoseMover
from simple_pid import PID
import matplotlib.pyplot as plt
import math

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


# ID delle varie parti del corpo
# 0-Nose 1-Neck 2-Rshoulder 3-RElbow 4-RWrist 5-LShoulder 6-LElbow  7-LWrist
# #Pose Recognition

def poseRecognizer(human):
    ##################################### DATI PER I PID #############################################################

    # Distanza tra le spalle , da passare al PID
    d_2_5 = distance(human.get_part(2).get_part_coordinate(),
                     human.get_part(5).get_part_coordinate()) * 960  # Per adattarlo alla misura del frame

    # Set target per face tracking da passare al PID  (coordinate del naso, distanza delle spalle)
    target = human.get_part(0).get_part_coordinate()
    target = (target[0] * 960, target[1] * 720, d_2_5)

    ##################################### CHECK CURRENT POSE #############################################################

    # Soglia delle distanza tra le varie parti del corpo
    th = 0.2

    # "ENTRAMBE LE MANI SUL CAPO"
    d_4_0 = distance(human.get_part(4).get_part_coordinate(), human.get_part(0).get_part_coordinate())
    d_7_0 = distance(human.get_part(7).get_part_coordinate(), human.get_part(0).get_part_coordinate())
    if d_4_0 < th and d_7_0 < th:
        return 1, target

        # "MANO DESTRA SULLA SPALLA SINSITRA"
    d_5_4 = distance(human.get_part(5).get_part_coordinate(), human.get_part(4).get_part_coordinate())
    if d_5_4 < th:
        return 2, target

        # "MANO SINISTRA SULLA SPALLA DESTRA"
    d_7_2 = distance(human.get_part(7).get_part_coordinate(), human.get_part(2).get_part_coordinate())
    if d_7_2 < th:
        return 3, target

        # ENTRAMBE LE MANI O MANO DESTRA/SINISTRA ALZATA (ALtezza delle orecchie)
    orcchie_y = human.get_part(16).get_part_coordinate()[1]
    mano_dx_y = human.get_part(4).get_part_coordinate()[1]
    mano_sx_y = human.get_part(7).get_part_coordinate()[1]
    if math.isnan(mano_dx_y) or math.isnan(mano_sx_y):
        return -3, target
    range_mano = np.float64(0.1)
    mano_dx_alzata = (orcchie_y + range_mano) >= mano_dx_y >= (orcchie_y - range_mano)
    mano_sx_alzata = (orcchie_y + range_mano) >= mano_sx_y >= (orcchie_y - range_mano)
    mani_alzate = mano_dx_alzata and mano_sx_alzata
    if mani_alzate:
        return 6, target
    elif mano_dx_alzata:
        return 5, target
    elif mano_sx_alzata:
        return 4, target

    # BRACCIA DISTESE O SINGOLO BRACCIO DISTESO
    range_braccio = np.float64(0.1)

    # Coordinate necessarie per il check della pose
    gomito_dx_y = human.get_part(3).get_part_coordinate()[1]
    spalla_dx_y = human.get_part(2).get_part_coordinate()[1]

    braccio_dx_disteso = (spalla_dx_y - range_braccio) <= gomito_dx_y <= (spalla_dx_y + range_braccio) and \
                         (spalla_dx_y - range_braccio) <= mano_dx_y <= (spalla_dx_y + range_braccio)

    if braccio_dx_disteso:
        return 7, target

    # Coordinate necessarie per il check della pose
    gomito_sx_y = human.get_part(6).get_part_coordinate()[1]
    spalla_sx_y = human.get_part(5).get_part_coordinate()[1]

    braccio_sx_disteso = (spalla_sx_y - range_braccio) <= gomito_sx_y <= (spalla_sx_y + range_braccio) and \
                         (spalla_sx_y - range_braccio) <= mano_sx_y <= (spalla_sx_y + range_braccio)

    braccia_distese = braccio_dx_disteso and braccio_sx_disteso

    if braccio_sx_disteso:
        return 8, target
    elif braccia_distese:
        return 9, target
    else:
        return -2, target


# FUNZIONE UTILIZZATA COME SEGNALE DI PARTENZA PER IL DRONE
def black_white_init(drone):
    counter = 0
    flag = True
    while counter < 3:
        frame = drone.get_frame_read().frame

        grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        (thresh, blackAndWhiteFrame) = cv2.threshold(grayFrame, 100, 255, cv2.THRESH_BINARY)

        # cv2.imshow('video bw', blackAndWhiteFrame)  #De-commentare se si vuole la visualizzazione
        mean = np.mean(blackAndWhiteFrame)
        if not flag and mean > 60:
            flag = True
        if flag and mean < 10:
            counter += 1
            flag = False

        print(f'Counter: {counter}\tMean: {mean}')

        if cv2.waitKey(1) == 27:
            break
    # cv2.destroyWindow('video bw')


pose = -1


def main():
    global pose

    fps_time = 0
    parser = argparse.ArgumentParser(description='Drone Pose Estimation and Face Tracking')
    parser.add_argument('--model', type=str, default='mobilenet_thin_432x368',
                        help='cmu_640x480 / cmu_640x360 / mobilenet_thin_432x368')
    parser.add_argument('--show-process', type=bool, default=False,
                        help='for debug purpose, if enabled, speed for inference is dropped.')
    args = parser.parse_args()
    w, h = model_wh(args.model)
    e = TfPoseEstimator(get_graph_path(args.model), target_size=(w, h))
    logger.debug('cam read+')

    num_to_pose = {
        -3: 'SKELETON NON CORRETTAMENTE RILEVATO',
        -2: 'NESSUNA POSE',
        -1: 'NESSUN UMANO',
        1: 'ENTRAMBE LE MANI SUL CAPO',
        2: 'MANO DESTRA SULLA SPALLA SINSITRA',
        3: 'MANO SINISTRA SULLA SPALLA DESTRA',
        4: 'MANO DESTRA ALZATA',
        5: 'MANO SINSITRA ALZATA',
        6: 'MANI IN ALTO',
        7: 'BRACCIO DX DISTESO',
        8: 'BRACCIO SX DISTESO',
        9: 'BRACCIA DISTESE'

    }

    # inizializzazione del drone e definizione dei dict per i messaggi sul frame e i comandi
    drone = tello.Tello()
    drone.RESPONSE_TIMEOUT = 5
    commands = {"yaw": 0, "roll": 0, "pitch": 0, "throttle": 0, "flip_left": 0, "flip_right": 0}
    states = {-1: "Avvio i motori...", 0: "IDLE - In attesa del segnale di partenza ... ",
              1: "POSE RECOGNITION E FACE TRACKING", 2: "RICERCA...", 4: "SPEGNIMENTO..",
              5: "IL PROGRAMMA TERMINERA' TRA 8s"}

    last_yaw = 0

    # Connessione con il drone e inizializzione pose_mover
    drone.connect()
    print(drone.get_battery())
    drone.streamon()
    pose_mover = PoseMover(drone)
    pose_to_do = -1
    state = 0

    while state != 5:
        image = drone.get_frame_read().frame
        if state == 0:
            print('INIT STATE')
            black_white_init(drone)  # funzione per il segnale di partenza
            state = -1
            time.sleep(5)
            drone.takeoff()
            state = 1
        elif state == 1:
            print('POSE RECOGNITION E FACE TRACKING')
            # Analisi del frame acquisito
            humans = e.inference(image)

            # Pose Recognition
            if len(humans) > 0:
                pose, target = poseRecognizer(humans[0])
            else:
                state = 2
                pose = -1  # se non ci sono persone all'interno del frame

            # Disegno dello skeleton sul frame
            image = TfPoseEstimator.draw_humans(image, humans, imgcopy=False)
            pose_to_do = pose
            if pose_to_do == 1:  # bisogna terminare
                state = 4
            elif pose_to_do == -1:  # sembra inutile
                commands = {"yaw": 0, "roll": 0, "pitch": 0, "throttle": 0, "flip_left": 0, "flip_right": 0}
            else:
                commands, image = pose_mover.move(pose_to_do, commands, target[0], target[1], target[2], image)

            cv2.putText(image,
                        num_to_pose[pose],
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 255, 0), 2)

            # Eseguo i comandi decisi dal pose_move e il face tracker
            if (commands["flip_left"] == 6):
                drone.flip_left()
                humans = []
            if (commands["flip_right"] == 6):
                drone.flip_right()
                humans = []
            else:
                drone.send_rc_control(commands["roll"], commands["pitch"], commands["throttle"], commands["yaw"])
            last_yaw = commands["yaw"]

        elif state == 2:
            print('RICERCA HUMAN')

            # Se il drone  girava a sinistra/destra(prima di entrare nello stato di ricerca), continua a girare a sinistra, poichè probabilmente la personaa è uscita dalla visuale da quella direzione
            if last_yaw != 0:
                drone.send_rc_control(0, 0, 0, last_yaw)
            else:
                drone.send_rc_control(0, 0, 0, 20)  # altrimenti cerco in modo casuale
            humans = e.inference(image)

            # Se trovo umani torno allo stato 1
            if len(humans) > 0:
                drone.send_rc_control(0, 0, 0, 0)
                last_yaw = 0
                state = 1

        elif state == 4:  # termina
            print('END')
            state = 5

        cv2.putText(image,
                    "FPS: %f" % (1.0 / (time.time() - fps_time)),
                    (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 255, 0), 2)

        cv2.putText(image,
                    f"STATE: {states[state]}",
                    (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 255, 0), 2)

        # SHOW
        cv2.imshow('Drone Cam', image)
        fps_time = time.time()

        if cv2.waitKey(1) == 32:
            drone.emergency()

    drone.land()
    time.sleep(8)
    print('PROGRAMMA TERMINATO')
    drone.end()


if __name__ == '__main__':
    main()
