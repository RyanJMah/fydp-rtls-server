import sys
import time
import logs
import threading
import queue
import numpy as np
from typing import List
from app_mqtt import TelemetryData
from mqtt_client import MqttClient, MqttMsg

from app_paths import AppPaths

sys.path.append(AppPaths.TESTS_DIR)
sys.path.append(AppPaths.TESTS_DIR)

from visualize_iphone import run, push_coordinates

logger = logs.init_logger(__name__)

anchor_distances: List[float] = [0.0, 0.0, 0.0]

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


def anchor_data_handler(client: MqttClient, msg: MqttMsg, id_: int):
    global anchor_distances

    data = TelemetryData.from_buffer_copy(msg.payload)

    if (id_ == 0):
        print(data.distance_mm / 10)

    if (data.status == 0):
        logger.info(f"got pub from anchor, id={id_}")
        # logger.info(f"got pub from anchor, id={id_}, {data}")

        anchor_distances[id_] = data.distance_mm / 10

        # avg_window = moving_avg_windows[id_]

        # if len(avg_window) == moving_avg_size:
        #     avg_distance = np.mean(avg_window)
        #     anchor_distances[id_] = avg_distance    # type: ignore

        #     avg_window.clear()

        # else:
        #     avg_window.append(data.distance_mm / 10)

def anchor_heartbat_handler(client: MqttClient, msg: MqttMsg, id_: int):
    logger.info(f"Received heartbeat from anchor {id_}")

def localization_thread():
    global anchor_distances

    anchor0 = np.array([0, 0, 0])
    anchor1 = np.array([74, 520, 0])
    anchor2 = np.array([564, 520, 0])

    while (1):
        r0, r1, r2 = anchor_distances

        # old one was 25
        r0 += 20
        r1 += 20
        r2 += 20

        coords, _ = trilateration(anchor0, anchor1, anchor2, r0, r1, r2)
        x, y, _ = coords

        logger.info(f"(x, y) = ({x}, {y})")

        push_coordinates(x, y, r0, r1, r2)

        time.sleep(0.1)

def main():
    client = MqttClient()

    client.connect("localhost", 1883)
    client.subscribe("/gl/anchor/<id>/data", anchor_data_handler)
    client.subscribe("/gl/anchor/<id>/heartbeat", anchor_heartbat_handler)
    client.start_mainloop()

    t = threading.Thread(target=localization_thread, daemon=True)
    t.start()

    run()

if __name__ == "__main__":
    main()
