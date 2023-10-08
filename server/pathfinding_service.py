import time
import json
from typing import Optional

import logs
from gl_conf import GL_CONF

import app_paths
import lib_pathfinding as cpp     # type: ignore

from mqtt_client import MqttClient, MqttMsg
from app_mqtt import SERVER_PATHFINDING_CONFIG_TOPIC

from abstract_service import AbstractService
from localization_service import LocalizationService_OutData
from debug_endpoint_service import DebugEndpointService, DebugEndpointData

logger = logs.init_logger(__name__)

class PathfindingService(AbstractService):
    def initialize(self):
        self.pf = cpp.Pathfinder()

        self.pf.load_navmesh( GL_CONF.navmesh_path )
        self.pf.set_navmesh_scale( GL_CONF.navmesh_to_real_life_scale )

        self.mqtt = MqttClient()

        self.mqtt.connect(GL_CONF.broker_address, GL_CONF.broker_port)
        self.mqtt.subscribe(SERVER_PATHFINDING_CONFIG_TOPIC, self.handle_config_publish)
        self.mqtt.start_mainloop()

        self.endpoint = [0, 0, 0]

    def handle_config_publish(self, client: MqttClient, msg: MqttMsg):
        """
        Every possible json input field

        {
            "endpoint": [x, y, z],
        }
        """
        inputs = json.loads( msg.payload.decode() )

        if "endpoint" in inputs.keys():
            self.endpoint = inputs["endpoint"]
            logger.info(f"changed endpoint to {self.endpoint}")


    def main(self, in_conn, out_conn):
        self.initialize()

        while (1):
            position_data: LocalizationService_OutData = in_conn.recv()     # type: ignore

            x = position_data.x
            y = position_data.y
            z = position_data.z

            # start = time.time()

            path = self.pf.find_path( (x, y, 0), self.endpoint  )

            # end = time.time()
            # logger.info(f"pathfinding runtime: {end - start}")

            if len(path) == 0:
                continue

            # Push to debug endpoint
            debug_data = DebugEndpointData( tag="path",
                                            data=path )

            DebugEndpointService.push(debug_data)
