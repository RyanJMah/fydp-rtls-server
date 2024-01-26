import time
import queue
from typing import Optional, List, Dict, Any
from dataclasses import dataclass, field

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


DISTANCE_FILTER_TIME_CONSTANT = 0.05
ANGLE_FILTER_TIME_CONSTANT    = 1


class _AnchorFilters:
    def __init__(self):
        self.distance_filter = LowPassFilter(DISTANCE_FILTER_TIME_CONSTANT)
        self.angle_filter    = LowPassFilter(ANGLE_FILTER_TIME_CONSTANT)


@dataclass
class AnchorRangingState:
    distance_cm       : float = 0.0
    angle_deg         : float = 0.0
    los               : bool  = False
    last_updated_time : float = 0.0

    @property
    def r(self):
        return self.distance_cm

    @property
    def phi(self):
        return self.angle_deg

@dataclass
class DIS_OutData:
    anchors: Dict[int, AnchorRangingState] = field( default_factory = lambda: {i: AnchorRangingState() for i in range(GL_CONF.num_anchors)} )


class DataIngestionService(AbstractService):
    def initialize(self):
        # init filters
        self.anchor_filters = [ _AnchorFilters() for _ in range(GL_CONF.num_anchors) ]

        # init "global" anchor ranging state, which is periodically sent to localization service
        self.out_data = DIS_OutData()

        # init mqtt
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

        self.out_data.anchors[aid].distance_cm       = filtered_distance * 100
        self.out_data.anchors[aid].angle_deg         = filtered_angle
        self.out_data.anchors[aid].los               = bool(data.los)
        self.out_data.anchors[aid].last_updated_time = time.time()


    def main(self, in_conn, out_conn):
        assert(in_conn is None)
        assert(out_conn is not None)

        self.initialize()

        logger.info("Starting data ingestion service...")

        self.mqtt_client.start_mainloop()

        timestamp = time.time()

        while (1):
            if (time.time() - timestamp) >= GL_CONF.update_period_secs:
                timestamp = time.time()
                out_conn.send( self.out_data )


