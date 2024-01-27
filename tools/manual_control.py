# NOTE: This script needs sudo

import sys
import time
import keyboard
import json

import includes
import logs
from gl_conf import GL_CONF
from mqtt_client import MqttClient, MqttMsg
from app_mqtt import SERVER_MANUAL_POSITION_CONTROL_TOPIC

logger = logs.init_logger(__name__)

SPEED = 7.5

UP = json.dumps(
    {
        "dir": "up",
        "speed": SPEED
    }
)
DOWN = json.dumps(
    {
        "dir": "down",
        "speed": SPEED
    }
)
LEFT = json.dumps(
    {
        "dir": "left",
        "speed": SPEED
    }
)
RIGHT = json.dumps(
    {
        "dir": "right",
        "speed": SPEED
    }
)


def main():
    mqtt_client = MqttClient()
    mqtt_client.connect(GL_CONF.broker_address, GL_CONF.broker_port)

    mqtt_client.start_mainloop()

    while (1):
        if keyboard.is_pressed("up"):
            # print("up")
            mqtt_client.publish(SERVER_MANUAL_POSITION_CONTROL_TOPIC, UP)

        elif keyboard.is_pressed("down"):
            # print("down")
            mqtt_client.publish(SERVER_MANUAL_POSITION_CONTROL_TOPIC, DOWN)


        if keyboard.is_pressed("left"):
            # print("left")
            mqtt_client.publish(SERVER_MANUAL_POSITION_CONTROL_TOPIC, LEFT)

        elif keyboard.is_pressed("right"):
            # print("right")
            mqtt_client.publish(SERVER_MANUAL_POSITION_CONTROL_TOPIC, RIGHT)
        
        # run every 10ms
        time.sleep(0.05)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)

