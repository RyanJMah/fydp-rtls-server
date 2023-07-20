import time
import test_paths

import logs
from mqtt_client import MqttClient, MqttMsg

logger = logs.init_logger(__name__)

def thing_id_msg_handler(client: MqttClient, msg: MqttMsg, id_: int):
    logger.info(f"{msg.topic}: received {msg.payload}")

    time.sleep(1)

    if id_ == 1:
        client.publish("/thing/2/msg", "b")

    elif id_ == 2:
        client.publish("/thing/deez", "c")


def thing_deez_handler(client: MqttClient, msg: MqttMsg):
    logger.info(f"{msg.topic}: received {msg.payload}")

    time.sleep(1)

    client.publish("/thing/1/msg", "a")


def main():
    client = MqttClient()

    client.connect("localhost", 1883)

    client.subscribe("/thing/<id>/msg", thing_id_msg_handler)
    client.subscribe("/thing/deez", thing_deez_handler)

    client.start_mainloop()

    time.sleep(0.5)

    client.publish("/thing/1/msg", "a")

    while (1):
        pass

if __name__ == "__main__":
    main()
