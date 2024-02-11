import time
import json
import threading
import numpy as np
import multiprocessing as mp
from dataclasses import dataclass
from typing import Tuple, List

import logs
from gl_conf import GL_CONF

import app_paths
import lib_pathfinding as cpp     # type: ignore

from mqtt_client import MqttClient, MqttMsg
from app_mqtt import SERVER_PATHFINDING_CONFIG_TOPIC

from lpf import LowPassFilter
from pid_controller import PIDController
from catmull_rom_splines import catmull_rom_chain

from abstract_service import AbstractService
from localization_service import LocalizationService_OutData
from outbound_data_service import OutboundDataService, OutboundData

logger = logs.init_logger(__name__)

TARGET_HEADING_TIME_CONSTANT = 0.025
TARGET_HEADING_ERROR_DEADZONE = 15 # degrees

DESTINATION_ACCEPTANCE_RADIUS = 50 # cm

@dataclass
class HapticsOptions:
    intensity: float
    heartbeat: bool
    done:      bool

class _PathfindingWorkerSlave(AbstractService):
    # Handles the expensive pathfinding calculations

    def __init__(self, in_conn, out_conn, daemon):
        super().__init__(in_conn, out_conn, daemon)
        self.initialize()

    def initialize(self):
        self.pf = cpp.Pathfinder()

        self.pf.load_navmesh( GL_CONF.navmesh_path )
        self.pf.set_navmesh_scale( GL_CONF.real_life_to_floorplan_png_scale )

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
                                          data={"val": path} )

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

        self.target_heading_pid = PIDController( kp = 0.01,
                                                 ki = 0.0,
                                                 kd = 0.002,
                                                 derivative_lpf_tau = 0.1 )

        self.haptics_pid = PIDController( kp = 0.0135,
                                          ki = 0.0,
                                          kd = 0.002,
                                          derivative_lpf_tau = 0.1 )

        # DO NOT CHANGE Kp EVER
        self.user_arrow_pid = PIDController( kp = 1,
                                             ki = 0.0,
                                             kd = 0.002,
                                             derivative_lpf_tau = 0.1 )
        ######################################################################################

        ######################################################################################
        self.endpoint = [0, 0, 0]
        self.perpendicular_distance_threshold = 100   # cm
        self.total_distance_threshold         = 200  # cm
        ######################################################################################


    def handle_config_publish(self, client: MqttClient, msg: MqttMsg):
        """
        Every possible json input field

        {
            "endpoint": [400, 200, 0],
            "perpendicular_distance_threshold": 200,
            "total_distance_threshold": 300,
            "kp": 0.5,
            "ki": 0.0,
            "kd": 0.0
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

    def determine_if_recalc_needed( self,
                                    curr_xy: Tuple[float, float],
                                    closest_index: int,
                                    closest_point_distance: float ):
        # Do nothing if the path is empty
        if len(self.path) == 0:
            return

        # Recalc if perpendicular distance is greater than threshold
        try:
            d = self.calc_perpendicular_distance(curr_xy, closest_index)

            if d > self.perpendicular_distance_threshold:
                self.recalc_path.set()
                return
        
        except ZeroDivisionError as e:
            logger.error(f"ZeroDivisionError in calc_perpendicular_distance: {e}")


        # Recalc if the total distance is greater than threshold
        if closest_point_distance > self.total_distance_threshold:
            self.recalc_path.set()
            return


    # TODO: reduce calls to this function by passing in the point as a parameter to functions which call it
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
            point1 = np.array(self.path[index - 1])
            point2 = np.array(self.path[index])

            tangent_vector = np.array(point2) - np.array(point1)

        elif index == 0:
            # Use the next point to calc the tangent at the beginning of the path
            point1 = np.array(self.path[index])
            point2 = np.array(self.path[index + 1])

            tangent_vector = np.array(point2) - np.array(point1)

        else:
            # Use the next and previous points to calc the tangent
            point1 = np.array(self.path[index - 1])
            point2 = np.array(self.path[index + 1])

            tangent_vector = np.array(point2) - np.array(point1)

        # Normalize the tangent vector
        # tangent_vector /= np.linalg.norm(tangent_vector)

        return tangent_vector, point2, point1


    def calc_perpendicular_distance(self, xy: Tuple[float, float], closest_index: int):
        x, y = xy

        xu, yu = x, y   # user's current position

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


    def calc_target_heading(self, xy: Tuple[float, float], closest_index: int) -> float:
        # target heading is the tangent angle + a steering term output
        # from the PID controller (to correct perpendicular distance error)

        x, y = xy

        # Ideal heading is the tangent angle
        tangent_vector, point2, point1  = self.calc_tangent_vector(closest_index)

        # Calculate the CTE (cross track error)
        err = self.calc_perpendicular_distance( xy, closest_index )
        
        # Calculate the steering term from the PID controller, tries to get the CTE to zero
        steering_angle = self.target_heading_pid.exec(err)

        # Determine the sign of the steering term based on the sign of the cross-product
        # of the tangent vector and the vector from the user's current position to the next point
        sign = np.sign( np.cross(tangent_vector, np.array(point2) - np.array((x, y))) )


        # Convert the tangent vector to an angle (heading)
        tangent_angle = np.arctan2(tangent_vector[1], tangent_vector[0])

        if tangent_angle < 0:
            tangent_angle += 2*np.pi

        # Add the steering term to the tangent angle
        target_heading = tangent_angle + sign*steering_angle

        return np.degrees( target_heading )


    def calc_haptics_options( self,
                              target_heading: float,
                              curr_heading: float,
                              curr_xy: Tuple[float, float] ) -> HapticsOptions:

        err = abs(target_heading - curr_heading)

        # Calculate the intensity of the haptic feedback
        intensity = self.haptics_pid.exec(err)

        # If the error is within the deadzone, heartbeat
        heartbeat = True if err < TARGET_HEADING_ERROR_DEADZONE else False

        # Done if the user is within the destination acceptance radius
        xyz = (curr_xy[0], curr_xy[1], 0)
        done = True if self.distance_between_points(xyz, self.endpoint) < DESTINATION_ACCEPTANCE_RADIUS else False

        # This is a bad place to put this, but I'm lazy
        if done:
            # reset the path
            self.path = []

            # send empty path to the application
            outbound_data = OutboundData( tag="path",
                                          data={"val": []} )
            OutboundDataService.push(outbound_data, is_debug_data=False)
            

        return HapticsOptions( intensity = intensity,
                               heartbeat = heartbeat,
                               done      = done )

    def calc_user_arrow_direction(self, target_heading: float, curr_heading: float) -> float:
        # Calculate the direction of the user arrow

        err = target_heading - curr_heading


        # Calculate the intensity of the user arrow
        steering = self.user_arrow_pid.exec(err)

        logger.info(f"target_heading: {target_heading:.2f}, curr_heading: {curr_heading:.2f}, err: {err:.2f}, steering: {steering:.2f}")

        return steering



    def main(self, in_conn, out_conn):
        assert(in_conn is not None)
        # assert(out_conn is not None)

        self.initialize()

        logger.info("Pathfinding service started")

        while (1):
            position_data: LocalizationService_OutData = in_conn.recv()     # type: ignore

            x       = position_data.x
            y       = position_data.y
            heading = position_data.heading


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
                closest_point, closest_index, d = self.find_closest_point_on_path( (x, y) )

                self.determine_if_recalc_needed( (x, y), closest_index, d )

                # start = time.time()
                target_heading = self.target_heading_filter.exec( self.calc_target_heading( (x, y), closest_index ) )
                # target_heading = self.calc_target_heading( (x, y), closest_index )
                # logger.info(f"target_heading: {target_heading:.2f}, time taken: {time.time() - start:.3f}s")

                haptics_options = self.calc_haptics_options( target_heading, heading, (x, y) )


                user_arrow = self.calc_user_arrow_direction( target_heading, heading )


                # Push target heading
                outbound_data = OutboundData( tag="target_heading",
                                              data={"val": target_heading} )

                OutboundDataService.push(outbound_data, is_debug_data=True)

                # Push haptics options
                outbound_data = OutboundData( tag="haptics_options",
                                              data=haptics_options )

                OutboundDataService.push(outbound_data, is_debug_data=False)

                # Push user arrow
                outbound_data = OutboundData( tag="user_arrow",
                                              data={"val": user_arrow} )
                
                OutboundDataService.push(outbound_data, is_debug_data=False)
