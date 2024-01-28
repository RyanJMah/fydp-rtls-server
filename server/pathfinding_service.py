import time
import json
import threading
import numpy as np
import multiprocessing as mp
from typing import Tuple, List

import logs
from gl_conf import GL_CONF

import app_paths
import lib_pathfinding as cpp     # type: ignore

from mqtt_client import MqttClient, MqttMsg
from app_mqtt import SERVER_PATHFINDING_CONFIG_TOPIC

from lpf import LowPassFilter
from catmull_rom_splines import catmull_rom_chain

from abstract_service import AbstractService
from localization_service import LocalizationService_OutData
from outbound_data_service import OutboundDataService, OutboundData

logger = logs.init_logger(__name__)

TARGET_HEADING_TIME_CONSTANT = 0.15

class _PathfindingWorkerSlave(AbstractService):
    # Handles the expensive pathfinding calculations

    def __init__(self, in_conn, out_conn, daemon):
        super().__init__(in_conn, out_conn, daemon)
        self.initialize()

    def initialize(self):
        self.pf = cpp.Pathfinder()

        self.pf.load_navmesh( GL_CONF.navmesh_path )
        self.pf.set_navmesh_scale( GL_CONF.navmesh_to_real_life_scale )

        self.pf.set_smoothing_factor(0.5)

        self.out_queue = mp.Queue(-1)   # using queue instead of pipe so we can use non-blocking polling

    def set_smoothing(self, smoothing_factor: float):
        self.pf.set_smoothing_factor(smoothing_factor)

    def new_path_available(self) -> bool:
        return not self.out_queue.empty()
    
    def get_new_path(self) -> List[ Tuple[float, float] ]:
        return self.out_queue.get()


    def main(self, in_conn, out_conn):
        assert(in_conn is not None)

        while (1):
            start_xyz, end_xyz = in_conn.recv()     # type: ignore

            path = self.pf.find_path( start_xyz, end_xyz )

            if len(path) == 0:
                # try again
                continue

            # Not using the z
            path = [(p[0], p[1],) for p in path]

            # Smooth the path via Catmull-Rom splines
            # path = catmull_rom_chain(path, points_per_joint=4)
            # logger.info(f"Path length: {len(path)}")

            # Push to the main service
            self.out_queue.put(path)

            # Push to application and debug endpoint
            outbound_data = OutboundData( tag="path",
                                          data=path )

            OutboundDataService.push(outbound_data, is_debug_data=False)
            OutboundDataService.push(outbound_data, is_debug_data=True)



class PathfindingService(AbstractService):
    def start_worker_pathfinding(self, start_xyz: Tuple[float, float, float], end_xyz: Tuple[float, float, float]):
        self._to_slave_conn1.send( (start_xyz, end_xyz,) )

    def initialize(self):
        ######################################################################################
        self._to_slave_conn1, self._to_slave_conn2 = mp.Pipe()

        self.worker = _PathfindingWorkerSlave(in_conn=self._to_slave_conn2, out_conn=None, daemon=True)
        self.worker.start()
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

        # self.point_indx_to_nav_to = 0

        self.path: List[ Tuple[float, float] ] = []

        self.target_heading_filter = LowPassFilter(tau=TARGET_HEADING_TIME_CONSTANT)
        ######################################################################################

        ######################################################################################
        self.endpoint = [0, 0, 0]
        self.perpendicular_distance_threshold = 50   # cm
        self.total_distance_threshold         = 125  # cm
        ######################################################################################


    def handle_config_publish(self, client: MqttClient, msg: MqttMsg):
        """
        Every possible json input field

        {
            "endpoint": [x, y, z],
            "distance_threshold": 200,
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

            self.recalc_path.set()

        if "perpendicular_distance_threshold" in inputs.keys():
            self.perpendicular_distance_threshold = inputs["perpendicular_distance_threshold"]
            logger.info(f"changed distance_threshold to {self.perpendicular_distance_threshold}")

        if "total_distance_threshold" in inputs.keys():
            self.total_distance_threshold = inputs["total_distance_threshold"]
            logger.info(f"changed total_distance_threshold to {self.total_distance_threshold}")


    def distance_between_points(self, xy1: Tuple[float], xy2: Tuple[float]):
        # Using NumPy arrays to represent the points
        point1 = np.array(xy1)
        point2 = np.array(xy2)

        # Calculate the Euclidean distance using NumPy functions
        distance = np.linalg.norm(point2 - point1)

        return distance

    def determine_if_recalc_needed(self, curr_xy: Tuple[float]):
        # Do nothing if the path is empty
        if len(self.path) == 0:
            return

        # Recalc if perpendicular distance is greater than threshold
        try:
            d = self.calc_perpendicular_distance(curr_xy[0], curr_xy[1])

            if d > self.perpendicular_distance_threshold:
                self.recalc_path.set()
                return
        
        except ZeroDivisionError as e:
            logger.error(f"ZeroDivisionError in calc_perpendicular_distance: {e}")


        # Recalc if the total distance is greater than threshold
        _, _, d = self.find_closest_point_on_path(curr_xy)

        if d > self.total_distance_threshold:
            self.recalc_path.set()
            return


    def find_closest_point_on_path(self, current_position):
        # Find the index of the closest point on the path

        distances = [self.distance_between_points(current_position, point) for point in self.path]

        closest_index = np.argmin(distances)

        # Return the closest point and its index, and its distance from the current position
        return self.path[closest_index], closest_index, distances[closest_index]

    def calc_tangent_vector(self, index):
        # Calculate the tangent vector at a given index on the path

        if index == len(self.path) - 1:
            # Use the previous point to calc the tangent at the end of the path
            tangent_vector = np.array(self.path[index]) - np.array(self.path[index - 1])

        elif index == 0:
            # Use the next point to calc the tangent at the beginning of the path
            tangent_vector = np.array(self.path[index + 1]) - np.array(self.path[index])

        else:
            # Use the next and previous points to calc the tangent
            tangent_vector = np.array(self.path[index + 1]) - np.array(self.path[index - 1])

        # Normalize the tangent vector
        # tangent_vector /= np.linalg.norm(tangent_vector)

        return tangent_vector

    def calc_tangent_angle(self, x, y):
        if not self.path:
            return 0.0  # Default heading if no path is available

        # Find the closest point on the path
        closest_point, closest_index, _ = self.find_closest_point_on_path((x, y))

        # Calculate the tangent vector at the closest point
        tangent_vector = self.calc_tangent_vector(closest_index)

        # Convert the tangent vector to an angle (heading)
        target_heading = np.arctan2(tangent_vector[1], tangent_vector[0])

        # Convert the angle from radians to degrees
        target_heading = np.degrees(target_heading)

        return target_heading


    def calc_perpendicular_distance(self, x, y):
        xu, yu = x, y   # user's current position

        _, closest_index, _ = self.find_closest_point_on_path((x, y))

        if closest_index == len(self.path) - 1:
            # Use the previous point to calc the tangent at the end of the path
            xp1, yp1 = self.path[closest_index - 1]
            xp2, yp2 = self.path[closest_index]

        else:
            # Use the next point to calc the tangent at the beginning of the path
            xp1, yp1 = self.path[closest_index]
            xp2, yp2 = self.path[closest_index + 1]


        # Interpolate line from xp1, yp1 to xp2, yp2 ==> y1 = m*x + b1

        if xp2 == xp1:
            # Handle the case where the line is vertical
            perpendicular_distance = np.abs(xu - xp1)
            return perpendicular_distance

        if yp2 == yp1:
            # Handle the case where the line is horizontal
            perpendicular_distance = np.abs(yu - yp1)
            return perpendicular_distance            

        m  = (yp2 - yp1) / (xp2 - xp1)
        b1 = yp1 - m*xp1

        # Interpolate line perpendicular to the above line which passes through (xu, yu) ==> y2 = -1/m*x + b2
        b2 = yu + (1/m)*xu

        # Solve for the intersection of the two lines
        x_intersection = (b2 - b1) / (m + 1/m)
        y_intersection = m*x_intersection + b1
        
        # Calculate the distance between the user's current position and the intersection point
        perpendicular_distance = self.distance_between_points( (xu, yu), (x_intersection, y_intersection) )

        return perpendicular_distance


    def calc_target_heading(self, x, y):
        # target heading is the tangent angle + a steering term output
        # from the PID controller (to correct perpendicular distance error)

        tangent_angle = self.calc_tangent_angle(x, y)

        # # for now
        # steering_term = 0

        # target_heading = tangent_angle + steering_term

        # return target_heading
        return tangent_angle


    def main(self, in_conn, out_conn):
        assert(in_conn is not None)
        # assert(out_conn is not None)

        self.initialize()

        logger.info("Pathfinding service started")

        while (1):
            position_data: LocalizationService_OutData = in_conn.recv()     # type: ignore

            x = position_data.x
            y = position_data.y
            # z = position_data.z

            self.determine_if_recalc_needed( (x, y) )

            # Recalc the path if needed
            if self.recalc_path.is_set():
                # Let the worker handle the pathfinding
                self.start_worker_pathfinding( (x, y, 0), self.endpoint )

                # Save the user's current position
                self.xy_at_last_path_calc = (x, y)

                # We've successfully recalcd the path, reset the event
                self.recalc_path.clear()

            # Update from worker if there's a new path
            if self.worker.new_path_available():
                self.path = self.worker.get_new_path()

            # Use the path to calculate the target heading
            if len(self.path) > 0:
                # start = time.time()
                target_heading = self.target_heading_filter.exec( self.calc_target_heading( x, y ) )
                # logger.info(f"target_heading: {target_heading:.2f}, time taken: {time.time() - start:.3f}s")

                outbound_data = OutboundData( tag="target_heading",
                                              data=target_heading )

                # Push to debug endpoint + app
                OutboundDataService.push(outbound_data, is_debug_data=False)
                OutboundDataService.push(outbound_data, is_debug_data=True)
