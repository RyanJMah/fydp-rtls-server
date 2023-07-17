import sys
import time
import logs
from app_mqtt import TelemetryData
from mqtt_client import MqttClient, MqttMsg

from app_paths import AppPaths
sys.path.append(AppPaths.TESTS_DIR)

logger = logs.init_logger(__name__)

def anchor_data_handler(client: MqttClient, msg: MqttMsg, id_: int):
    data = TelemetryData.from_buffer_copy(msg.payload)

    logger.info(f"got pub from anchor, id={id_}, {data}")

def anchor_heartbat_handler(client: MqttClient, msg: MqttMsg, id_: int):
    logger.info(f"Received heartbeat from anchor {id_}")

def main():
    client = MqttClient()

    client.connect("localhost", 1883)
    client.subscribe("/gl/anchor/<id>/data", anchor_data_handler)
    client.subscribe("/gl/anchor/<id>/heartbeat", anchor_heartbat_handler)
    client.start_mainloop()

    while (1):
        pass

if __name__ == "__main__":
    main()
