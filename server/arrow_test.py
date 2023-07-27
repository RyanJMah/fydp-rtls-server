import sys
import time
import logs
import threading
import queue
import numpy as np
from typing import List
from app_mqtt import TelemetryData
from mqtt_client import MqttClient, MqttMsg

import random

def main():
    client = MqttClient()

    client.connect("localhost", 1883)
    client.start_mainloop()

    table = {
        0: "straight",
        1: "left",
        2: "right"
    }

    while (1):
        tmp = random.randint(0, 2)
        client.publish("gl/user/0/pathing", f'{{"direction": \"{table[tmp]}\"}}')
        time.sleep(3)

if __name__ == "__main__":
    main()
