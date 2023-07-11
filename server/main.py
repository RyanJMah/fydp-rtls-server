import time
import logs
from mqtt_client import MqttClient, MqttMsg

from mqtt_client import _test_mqtt_client

logger = logs.init_logger(__name__)

def main():
    _test_mqtt_client()


if __name__ == "__main__":
    main()
