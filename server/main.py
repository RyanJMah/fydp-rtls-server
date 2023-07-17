import sys
import time
import logs
from mqtt_client import MqttClient, MqttMsg

from app_paths import AppPaths
sys.path.append(AppPaths.TESTS_DIR)

logger = logs.init_logger(__name__)

def anchor_data_handler(client: MqttClient, msg: MqttMsg, id_: int):
    print(f"got pub from anchor, id={id_}")

def main():
    client = MqttClient()

    client.connect("localhost", 1883)
    client.subscribe("/gl/anchor/<id>/data", anchor_data_handler)
    client.start_mainloop()

    while (1):
        pass

if __name__ == "__main__":
    main()
