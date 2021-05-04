from djitellopy import tello
import cv2
from simple_pid import PID
import math


class PoseMover:
    def __init__(self, drone: tello.Tello):

        self.drone = drone
        self.target = [960 / 2, 720 * 0.35, 0]  # Messo solo per sicurezza --> Quando non c'è naso, sto fermo al centro

        # SetPoint
        self.ref_x = int(960 / 2)
        self.ref_y = int(720 * 0.35)

        # SetPoint - Range in cui la distanza delle spalle deve essere compresa
        self.ref_z_up = 240
        self.ref_z_down = 230

        # Inizializzazione dei PID con gli opportuni coefficienti
        self.pid_yaw = PID(0.12, 0, 0.00, setpoint=0, output_limits=(-100, 100))
        self.pid_throttle = PID(0.12, 0, 0.00, setpoint=0, output_limits=(-80, 100))
        self.pid_pitch = PID(0.25, 0, 0.00, setpoint=0, output_limits=(-80, 100))

    def move(self, mv, commands, x, y, z, imageft):

        # Se nessun human è rilevato x e y sono nan --> controllo
        if not math.isnan(x):
            self.target[0] = x
            self.target[1] = y
            self.target[2] = z

        # FACE TRACKING AND COMMAND SET
        # Calcolo degli errori
        xoff = int(self.target[0] - self.ref_x)
        yoff = int(self.ref_y - self.target[1])
        zoff = 0
        if self.target[2] == 0:
            zoff = 0
        elif self.target[2] > self.ref_z_up:
            zoff = int(self.target[2] - self.ref_z_up)
        elif self.target[2] < self.ref_z_down:
            zoff = -int(self.ref_z_down - self.target[2])

        # Disegno sul frame
        cv2.circle(imageft, (self.ref_x, self.ref_y), 15, (250, 150, 0), 1, cv2.LINE_AA)
        cv2.arrowedLine(imageft, (self.ref_x, self.ref_y), (int(self.target[0]), int(self.target[1])), (250, 150, 0), 6)

        # Assegno le velocità calcolate dal PID alle variabili di controllo del drone
        commands["yaw"] = int(-self.pid_yaw(xoff))
        commands["throttle"] = int(-self.pid_throttle(yoff))
        commands["pitch"] = -int(-self.pid_pitch(zoff))

        # In base alla pose rilevata imposto le variabili di controllo
        if mv == 0:
            print('take_photo')
        elif mv == 2:
            commands["roll"] = 30  # Vado a destra
        elif mv == 3:
            commands["roll"] = -30  # Vado a sinistra
        elif mv == 4:
            commands["flip_left"] += 1  # flip sinistra
        elif mv == 5:
            commands["flip_right"] += 1  # flip destra
        elif mv == 6:
            self.take_photo()  # take photo
        else:
            commands["roll"] = 0  # Se non c'è nessuna pose azzero le variabili di controllo
            commands["flip_left"] = 0
            commands["flip_right"] = 0

        return commands, imageft  # ritorno i valori delle velocità da passare al drone/azione e il frame

    def take_photo(self):
        """Take a photo from video"""
        frame_read = self.drone.get_frame_read()
        cv2.imwrite("picture.png", frame_read.frame)
