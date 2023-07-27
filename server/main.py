import sys
import time
import logs
import threading
import queue
import json
import numpy as np
from dataclasses import dataclass
from typing import List
from app_mqtt import TelemetryData
from mqtt_client import MqttClient, MqttMsg

from app_paths import AppPaths

sys.path.append(AppPaths.TESTS_DIR)
sys.path.append(AppPaths.TESTS_DIR)

from visualize_iphone import run, push_coordinates

logger = logs.init_logger(__name__)

ANCHOR_0_COORDINATES = (615, 0, 266)
ANCHOR_1_COORDINATES = (615, 520, 279)
ANCHOR_2_COORDINATES = (0, 0, 273)
ANCHOR_3_COORDINATES = (0, 520, 273)

class LowPassFilter:
    """
    First order low pass filter, discretized via trapazoidal rule,
    with a sample time of 5.5Hz
    """

    def __init__(self):
        self.tau = 0.075

        self.y_n_minus_1 = 0
        self.u_n_minus_1 = 0

    def exec(self, u):
        y = ( 1/(11*self.tau + 1) ) * \
            (u + self.u_n_minus_1 - (1 - 11*self.tau)*self.y_n_minus_1)

        # y = 0.8377*self.u_n_minus_1 + 0.1623*self.y_n_minus_1

        self.y_n_minus_1 = y
        self.u_n_minus_1 = u

        return y

class Anchor:
    def __init__(self):
        self.distance_cm = 0.0
        self.iphone_angle_degrees = 0.0
        self.iphone_angle_valid = False
        self.los = False

        self.distance_filter = LowPassFilter()
        self.angle_filter    = LowPassFilter()

g_anchors = [Anchor(), Anchor(), Anchor(), Anchor()]

def trilateration(P1, P2, P3, r1, r2, r3):
    p1 = np.array([0, 0, 0])
    p2 = np.array([P2[0] - P1[0], P2[1] - P1[1], P2[2] - P1[2]])
    p3 = np.array([P3[0] - P1[0], P3[1] - P1[1], P3[2] - P1[2]])
    v1 = p2 - p1
    v2 = p3 - p1

    Xn = (v1)/np.linalg.norm(v1)

    tmp = np.cross(v1, v2)

    Zn = (tmp)/np.linalg.norm(tmp)

    Yn = np.cross(Xn, Zn)

    i = np.dot(Xn, v2)
    d = np.dot(Xn, v1)
    j = np.dot(Yn, v2)

    X = ((r1**2)-(r2**2)+(d**2))/(2*d)
    Y = (((r1**2)-(r3**2)+(i**2)+(j**2))/(2*j))-((i/j)*(X))
    Z1 = np.sqrt(max(0, r1**2-X**2-Y**2))
    Z2 = -Z1

    K1 = P1 + X * Xn + Y * Yn + Z1 * Zn
    K2 = P1 + X * Xn + Y * Yn + Z2 * Zn
    return K1,K2


def anchor_data_handler(client: MqttClient, msg: MqttMsg, aid: int):
    global g_anchors

    data = TelemetryData.from_buffer_copy(msg.payload)

    if (data.status == 0):
        filtered_distance = g_anchors[aid].distance_filter.exec(data.distance_mm / 10)
        g_anchors[aid].distance_cm = filtered_distance

def ios_data_handler(client: MqttClient, msg: MqttMsg, uid: int, aid:int):
    global g_anchors

    data = json.loads(msg.payload.decode())

    print(data)


def localization_thread():
    global g_anchors

    anchor0_coordinates = np.array([*ANCHOR_0_COORDINATES])
    anchor1_coordinates = np.array([*ANCHOR_1_COORDINATES])
    anchor2_coordinates = np.array([*ANCHOR_2_COORDINATES])

    a0, a1, a2, a3 = g_anchors

    while (1):
        r0, phi0 = a0.distance_cm, a0.iphone_angle_degrees
        r1, phi1 = a1.distance_cm, a1.iphone_angle_degrees
        r2, phi2 = a2.distance_cm, a2.iphone_angle_degrees
        r3, phi3 = a3.distance_cm, a3.iphone_angle_degrees

        # old one was 25
        r0 += 10
        r1 += 10
        r2 += 10
        r3 += 10

        coords, _ = trilateration( anchor0_coordinates,
                                   anchor1_coordinates,
                                   anchor2_coordinates,
                                   r0,
                                   r1,
                                   r2 )
        x, y, _ = coords

        # logger.info(f"(x, y) = ({x}, {y})")

        push_coordinates(x, y, r0, phi0, r1, phi1, r2, phi2)

        time.sleep(0.1)

def main():
    client = MqttClient()

    client.connect("localhost", 1883)

    client.subscribe("gl/anchor/<id>/data", anchor_data_handler)
    client.subscribe("gl/user/<uid>/data/<aid>", ios_data_handler)

    client.start_mainloop()

    t = threading.Thread(target=localization_thread, daemon=True)
    t.start()

    run()

if __name__ == "__main__":
    main()
