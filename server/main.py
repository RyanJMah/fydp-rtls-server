import sys
import time
import logs
from mqtt_client import MqttClient, MqttMsg

from app_paths import AppPaths
sys.path.append(AppPaths.TESTS_DIR)

import mqtt_client_test

logger = logs.init_logger(__name__)

def main():
    mqtt_client_test.main()


if __name__ == "__main__":
    main()
