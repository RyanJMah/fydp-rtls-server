import json
import queue
import threading
import numpy as np
from typing import Tuple

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
        ######################################################################################
        self.pf = cpp.Pathfinder()

        self.pf.load_navmesh( GL_CONF.navmesh_path )
        self.pf.set_navmesh_scale( GL_CONF.navmesh_to_real_life_scale )
        ######################################################################################

        ######################################################################################
        self.mqtt = MqttClient()

        self.mqtt.connect(GL_CONF.broker_address, GL_CONF.broker_port)
        self.mqtt.subscribe(SERVER_PATHFINDING_CONFIG_TOPIC, self.handle_config_publish)
        self.mqtt.start_mainloop()
        ######################################################################################

        ######################################################################################
        self.recalc_path = threading.Event()
        self.recalc_path.clear()

        self.xy_at_last_path_calc = (0, 0)
        ######################################################################################

        ######################################################################################
        self.endpoint           = [0, 0, 0]
        self.distance_threshold = 35   # cm
        ######################################################################################


    def handle_config_publish(self, client: MqttClient, msg: MqttMsg):
        """
        Every possible json input field

        {
            "endpoint": [x, y, z],
            "distance_threshold": 200
        }
        """
        try:
            inputs = json.loads( msg.payload.decode() )

        except json.JSONDecodeError:
            logger.error(f"Failed to decode json from {msg.payload.decode()}")
            return

        if "endpoint" in inputs.keys():
            old_endpoint = self.endpoint
            self.endpoint = inputs["endpoint"]

            logger.info(f"changed endpoint to {self.endpoint}")

            # recalc path if endpoint changes
            if old_endpoint != self.endpoint:
                self.recalc_path.set()


        if "distance_threshold" in inputs.keys():
            self.distance_threshold = inputs["distance_threshold"]
            logger.info(f"changed distance_threshold to {self.distance_threshold}")


    def distance_between_points(self, xy1: Tuple[float], xy2: Tuple[float]):
        # Using NumPy arrays to represent the points
        point1 = np.array(xy1)
        point2 = np.array(xy2)

        # Calculate the Euclidean distance using NumPy functions
        distance = np.linalg.norm(point2 - point1)

        return distance

    def determine_if_recalc_needed(self, curr_xy: Tuple[float]):
        d = self.distance_between_points(curr_xy, self.xy_at_last_path_calc)

        if d > self.distance_threshold:
            self.recalc_path.set()


    def async_find_path(self, start: Tuple[float], end: Tuple[float]):
        # Kick-off a thread to calculate the path

        t = threading.Thread(target = self._find_path, args = (start, end,))
        t.start()


    def main(self, in_conn, out_conn):
        assert(in_conn is not None)
        # assert(out_conn is not None)

        self.initialize()

        logger.info("Pathfinding service started")

        # Algorithm:
        #
        # 1. Calculate the path from the user's current position to the endpoint, store the user's current position.
        # 2. If the user's position deviates from the stored position past a certain threshold, recalculate the path.
        #
        # Some edgecases:
        #
        # 1. If the endpoint changes, recalculate the path

        while (1):
            position_data: LocalizationService_OutData = in_conn.recv()     # type: ignore

            x = position_data.x
            y = position_data.y
            z = position_data.z

            self.determine_if_recalc_needed( (x, y) )

            if self.recalc_path.is_set():
                # Recalculate the path...

                path = self.pf.find_path( (x, y, 0), self.endpoint )

                if len(path) == 0:
                    # try again
                    continue

                # Save the user's current position
                self.xy_at_last_path_calc = (x, y)

                # We've successfully recalculated the path, reset the event
                self.recalc_path.clear()


                # Push to debug endpoint
                debug_data = DebugEndpointData( tag="path",
                                                data=path )
                DebugEndpointService.push(debug_data)

