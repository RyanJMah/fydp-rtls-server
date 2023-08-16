import time
import queue

from app_mqtt import (
    AnchorTelemetryData,
    IOS_TelemetryData,
    BROKER_ADDRESS,
    BROKER_PORT,
    ANCHOR_DATA_TOPIC,
    IOS_DATA_TOPIC
)
from mqtt_client import MqttClient, MqttMsg
from abstract_service import AbstractService

class DataIngestionService(AbstractService):
    def __init__(self, in_queue: queue.Queue, out_queue: queue.Queue):
        super().__init__(in_queue, out_queue)

        self.mqtt_client = MqttClient()

    def anchor_data_handler(self, client: MqttClient, msg: MqttMsg, aid: int):
        data = AnchorTelemetryData.from_buffer_copy(msg.payload)

        if (data.status == 0):
            self.out_queue.put(data)

    def ios_data_handler(self, client: MqttClient, msg: MqttMsg, uid: int, aid:int):
        self.out_queue.put( IOS_TelemetryData.from_buffer_copy(msg.payload) )

    def mainloop(self):
        self.mqtt_client.connect(BROKER_ADDRESS, BROKER_PORT)

        self.mqtt_client.subscribe(ANCHOR_DATA_TOPIC, self.anchor_data_handler)
        self.mqtt_client.subscribe(IOS_DATA_TOPIC, self.ios_data_handler)

        self.mqtt_client.start_mainloop()

        while (1):
            time.sleep(60)

