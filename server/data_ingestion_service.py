import time
import queue
from typing import Optional, List, Any
from dataclasses import dataclass

import logs
from app_mqtt import (
    AnchorTelemetryData,
    IOS_TelemetryData,
    ANCHOR_DATA_TOPIC,
    IOS_DATA_TOPIC
)
from mqtt_client import MqttClient, MqttMsg
from abstract_service import AbstractService
from lpf import LowPassFilter
from gl_conf import GL_CONF


logger = logs.init_logger(__name__)


DISTANCE_FILTER_TIME_CONSTANT = 0.175
ANGLE_FILTER_TIME_CONSTANT    = 1


class _AnchorFilters:
    def __init__(self):
        self.distance_filter = LowPassFilter(DISTANCE_FILTER_TIME_CONSTANT)
        self.angle_filter    = LowPassFilter(ANGLE_FILTER_TIME_CONSTANT)


@dataclass
class DIS_OutData:
    aid         : int
    uid         : int
    distance_cm : float
    angle_deg   : float
    los         : bool


class DataIngestionService(AbstractService):
    def __init__(self, in_conn: Any, out_conn: Any):
        super().__init__(in_conn, out_conn)

        self.mqtt_client: Optional[MqttClient] = None
        self.anchor_filters: List[_AnchorFilters] = []


    def init_filters(self):
        self.anchor_filters = [ _AnchorFilters() for _ in range(GL_CONF.num_anchors) ]


    def init_mqtt(self):
        self.mqtt_client = MqttClient()

        self.mqtt_client.connect(GL_CONF.broker_address, GL_CONF.broker_port)

        self.mqtt_client.subscribe(ANCHOR_DATA_TOPIC, self.anchor_data_handler)
        self.mqtt_client.subscribe(IOS_DATA_TOPIC, self.ios_data_handler)


    def anchor_data_handler(self, client: MqttClient, msg: MqttMsg, aid: int):
        # data = AnchorTelemetryData.from_buffer_copy(msg.payload)

        # if (data.status == 0):
            # self.in_queue.put(data)

        pass


    def ios_data_handler(self, client: MqttClient, msg: MqttMsg, uid: int, aid:int):
        try:
            data = IOS_TelemetryData.from_buffer_copy(msg.payload)
        except Exception as e:
            logger.error(f"Failed to parse IOS_TelemetryData: {e}")
            return

        filtered_distance = self.anchor_filters[aid].distance_filter.exec(data.distance_m)
        filtered_angle    = self.anchor_filters[aid].angle_filter.exec(data.azimuth_deg)

        out_data = DIS_OutData( aid         = aid,
                                uid         = uid,
                                distance_cm = filtered_distance * 100,
                                angle_deg   = filtered_angle,
                                los         = data.los )

        self.out_conn.send(out_data)    # type: ignore


    def main(self, in_conn, out_conn):
        assert(in_conn is None)
        assert(out_conn is not None)

        self.init_filters()
        self.init_mqtt()

        self.mqtt_client.run_mainloop()

        # assert(self.mqtt_client is not None)

        # self.mqtt_client.run_mainloop()

