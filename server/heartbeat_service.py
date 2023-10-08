import time
import threading
from typing import Dict

import logs
from mqtt_client import MqttClient, MqttMsg
from abstract_service import AbstractService
from gl_conf import GL_CONF
from app_mqtt import (
    ANCHOR_HEARTBEAT_TOPIC,
    SERVER_HEARTBEAT_TOPIC
)

logger = logs.init_logger(__name__)

HEARTBEAT_CHECKING_PERIODICITY = 10 # seconds
HEARTBEAT_DOWNTIME_THRESHOLD   = 10 # seconds
HEARTBEAT_SEND_PERIODICITY     = 5  # seconds

class HeartbeatService(AbstractService):
    def initialize(self):
        self.anchor_heartbeat_timestamps = {}

        self.mqtt = MqttClient()

        self.mqtt.connect(GL_CONF.broker_address, GL_CONF.broker_port)
        self.mqtt.subscribe(ANCHOR_HEARTBEAT_TOPIC, self.anchor_heartbeat_handler)


    def anchor_heartbeat_handler(self, client: MqttClient, msg: MqttMsg, aid: int):
        self.anchor_heartbeat_timestamps[aid] = time.time()


    def check_for_missed_heartbeats(self):
        for aid, timestamp in self.anchor_heartbeat_timestamps.items():
            if timestamp >= HEARTBEAT_DOWNTIME_THRESHOLD:
                logger.warning(f"Anchor {aid} missed heartbeat")


    def send_heartbeat(self):
        self.mqtt.publish(SERVER_HEARTBEAT_TOPIC, r'{status: "online"}')


    def main(self, in_conn, out_conn):
        self.initialize()

        t1 = threading.Timer(HEARTBEAT_CHECKING_PERIODICITY, self.check_for_missed_heartbeats)
        t2 = threading.Timer(HEARTBEAT_SEND_PERIODICITY,     self.send_heartbeat)

        self.mqtt.start_mainloop()

        t1.start()
        t2.start()

        while (1):
            pass
