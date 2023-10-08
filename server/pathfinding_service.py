import time
from typing import Optional

import logs
from gl_conf import GL_CONF

import app_paths
import lib_pathfinding as cpp     # type: ignore

from mqtt_client import MqttClient

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


    def main(self, in_conn, out_conn):
        self.initialize()

        timestamp = time.time()
    
        while (1):
            position_data: LocalizationService_OutData = in_conn.recv()     # type: ignore

            x = position_data.x
            y = position_data.y
            z = position_data.z

            # start = time.time()

            path = self.pf.find_path( (x, y, 0), (500, 500, 0)  )

            # end = time.time()
            # logger.info(f"pathfinding runtime: {end - start}")

            if len(path) == 0:
                continue

            # Push to debug endpoint
            debug_data = DebugEndpointData( tag="path",
                                            data=path )

            DebugEndpointService.push(debug_data)
