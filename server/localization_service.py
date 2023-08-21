import time
import socket
import pickle
import numpy as np
import threading
import atexit
from typing import Optional, List, Dict, Any
from dataclasses import dataclass

import logs
from gl_conf import GL_CONF
from lpf import LowPassFilter
from abstract_service import AbstractService
from data_ingestion_service import DIS_OutData, AnchorRangingState

from KalmanFilter import KalmanFilter as kf
from KalmanFilter import initConstVelocityKF as initConstVel
from KalmanFilter import initConstAccelerationKF as initConstAcc

logger = logs.init_logger(__name__)

@dataclass
class LocalizationService_DebugData:
    x         : float = 0.0
    y         : float = 0.0
    z         : float = 0.0
    angle_deg : float = 0.0

    r0: float = 0.0
    r1: float = 0.0
    r2: float = 0.0
    r3: float = 0.0

    phi0: float = 0.0
    phi1: float = 0.0
    phi2: float = 0.0
    phi3: float = 0.0

    los0: bool = False
    los1: bool = False
    los2: bool = False
    los3: bool = False

    critical_anchor: int = 0


@dataclass
class LocalizationService_OutData:
    x         : float = 0.0
    y         : float = 0.0
    z         : float = 0.0
    angle_deg : float = 0.0


class LocalizationService(AbstractService):
    def init_kalman_filter(self):
        A = np.zeros((9,9))
        H = np.zeros((3,9))

        self.kf = kf(A, H, 0)

        # A, B, H, Q, R, P_0, x_0 = initConstVel()
        A, B, H, Q, R, P_0, x_0 = initConstAcc()

        self.kf.assignSystemParameters(A, B, H, Q, R, P_0, x_0)


    def initialize(self):
        # init "global" user location, which is periodically sent to pathfinding service
        self.out_data = LocalizationService_OutData()

        # init "global" debug data, for the debug endpoint
        self.debug_data = LocalizationService_DebugData()

        # event that fires when new data is available, used to signal to debug endpoint
        self.new_data_evt = threading.Event()

        self.init_kalman_filter()


    def multilateration(self, anchors: Dict[int, AnchorRangingState]):
        """
        Multilateration via weighted least squares estimation
            - Reference equation 5 of https://ieeexplore.ieee.org/document/8911811

        Least squares estimate of a potentially undersolved system of equations is A^T * A * x = A^T * b
            - Refence: https://textbooks.math.gatech.edu/ila/least-squares.html
        """

        r0 = anchors[0].r
        r1 = anchors[1].r
        r2 = anchors[2].r
        r3 = anchors[3].r

        x0 = GL_CONF.anchor_coords[0][0]
        y0 = GL_CONF.anchor_coords[0][1]
        z0 = GL_CONF.anchor_coords[0][2]

        x1 = GL_CONF.anchor_coords[1][0]
        y1 = GL_CONF.anchor_coords[1][1]
        z1 = GL_CONF.anchor_coords[1][2]

        x2 = GL_CONF.anchor_coords[2][0]
        y2 = GL_CONF.anchor_coords[2][1]
        z2 = GL_CONF.anchor_coords[2][2]

        x3 = GL_CONF.anchor_coords[3][0]
        y3 = GL_CONF.anchor_coords[3][1]
        z3 = GL_CONF.anchor_coords[3][2]

        A = np.array([
            [x1-x0, y1-y0, z1-z0],
            [x2-x0, y2-y0, z2-z0],
            [x3-x0, y3-y0, z3-z0]
        ])

        b = 0.5 * np.array([
            [r0**2 - r1**2 + (x1**2 + y1**2 + z1**2) - (x0**2 + y0**2 + z0**2)],
            [r0**2 - r2**2 + (x2**2 + y2**2 + z2**2) - (x0**2 + y0**2 + z0**2)],
            [r0**2 - r3**2 + (x3**2 + y3**2 + z3**2) - (x0**2 + y0**2 + z0**2)]
        ])

        tmp = np.linalg.inv( np.matmul(A.T, A) )
        tmp = np.matmul(tmp, A.T)
        x   = np.matmul(tmp, b)

        return x[0], x[1], x[2]


    def trilateration(self, a1, a2, a3, r1, r2, r3):
        P1 = GL_CONF.anchor_coords[a1]
        P2 = GL_CONF.anchor_coords[a2]
        P3 = GL_CONF.anchor_coords[a3]

        # r1 += 10
        # r2 += 10
        # r3 += 10

        p1 = np.array([0, 0, 0])
        p2 = np.array([P2[0] - P1[0], P2[1] - P1[1], P2[2] - P1[2]])
        p3 = np.array([P3[0] - P1[0], P3[1] - P1[1], P3[2] - P1[2]])
        v1 = p2 - p1
        v2 = p3 - p1

        Xn = (v1)/np.linalg.norm(v1)

        tmp = np.cross(v1, v2)

        Zn = (tmp)/np.linalg.norm(tmp)

        Yn = np.cross(Xn, Zn)

        i = np.dot(Xn, v2)
        d = np.dot(Xn, v1)
        j = np.dot(Yn, v2)

        X = ((r1**2)-(r2**2)+(d**2))/(2*d)
        Y = (((r1**2)-(r3**2)+(i**2)+(j**2))/(2*j))-((i/j)*(X))
        Z1 = np.sqrt(max(0, r1**2-X**2-Y**2))
        Z2 = -Z1

        K1 = P1 + X * Xn + Y * Yn + Z1 * Zn
        K2 = P1 + X * Xn + Y * Yn + Z2 * Zn
        return K1,K2


    # determine which anchors to use for trilateration based on LOS conditions
    def select_anchors_for_trilateration(self, anchors: Dict[int, AnchorRangingState]):
        # (anchor3)            (anchor1)
        #                               
        #                               
        #                               
        # (anchor2)            (anchor0)

        los0 = anchors[0].los
        los1 = anchors[1].los
        los2 = anchors[2].los
        los3 = anchors[3].los

        los = [
            anchors[0].los,
            anchors[1].los,
            anchors[2].los,
            anchors[3].los
        ]

        r = [
            anchors[0].r,
            anchors[1].r,
            anchors[2].r,
            anchors[3].r
        ]

        phi = [
            anchors[0].phi,
            anchors[1].phi,
            anchors[2].phi,
            anchors[3].phi
        ]

        #################################################
        # DEFAULT VALUES
        trilat_a1 = 1
        trilat_r1 = r[1]

        trilat_a2 = 3
        trilat_r2 = r[3]

        trilat_a3 = 2
        trilat_r3 = r[2]
        #################################################

        num_los = len([l for l in [los0, los1, los2, los3] if l])

        if num_los > 1:
            best_anchor = np.argmin(np.abs(phi))

            for i in range(len(los)):
                if i != best_anchor:
                    los[i] = False

        if los[0]:
            trilat_a1 = 2
            trilat_r1 = r[2]

            trilat_a2 = 0
            trilat_r2 = r[0]

            trilat_a3 = 1
            trilat_r3 = r[1]

        elif los[1]:
            trilat_a1 = 0
            trilat_r1 = r[0]

            trilat_a2 = 1
            trilat_r2 = r[1]

            trilat_a3 = 2
            trilat_r3 = r[2]

        elif los[2]:
            trilat_a1 = 3
            trilat_r1 = r[3]

            trilat_a2 = 2
            trilat_r2 = r[2]

            trilat_a3 = 0
            trilat_r3 = r[0]

        elif los[3]:
            trilat_a1 = 1
            trilat_r1 = r[1]

            trilat_a2 = 3
            trilat_r2 = r[3]

            trilat_a3 = 2
            trilat_r3 = r[2]

        return ( trilat_a1, trilat_a2, trilat_a3, \
                 trilat_r1, trilat_r2, trilat_r3 )


    def debug_endpoint_thread(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # this is so we can restart the program without waiting for the socket to timeout
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        sock.bind( (GL_CONF.loc_debug_endpoint.host, GL_CONF.loc_debug_endpoint.port) )
        sock.listen()

        logger.info("Debug endpoint listening on %s:%d" % (GL_CONF.loc_debug_endpoint.host, GL_CONF.loc_debug_endpoint.port))

        conn, addr = sock.accept()
        with conn:
            logger.info("client connected to debug endpoint...")

            while (1):
                self.new_data_evt.wait()
                self.new_data_evt.clear()

                conn.send( pickle.dumps(self.debug_data) )


    def main(self, in_conn, out_conn):
        assert(in_conn is not None)
        assert(out_conn is not None)

        self.initialize()

        if GL_CONF.loc_debug_endpoint.enabled:
            threading.Thread(target=self.debug_endpoint_thread, daemon=True).start()

        while (1):
            anchor_data: DIS_OutData = self.in_conn.recv()     # type: ignore

            anchors = anchor_data.anchors

            x, y, z = self.multilateration( anchors )

            # Run coords through Kalman filter
            meas = np.array([x, y, z])
            meas.shape = (len(meas), 1)
            self.kf.performKalmanFilter(np.array(meas), 0)

            kf_x = self.kf.x_m[0]
            kf_y = self.kf.x_m[1]
            kf_z = self.kf.x_m[2]

            # Copy data to output variable
            self.out_data.x         = kf_x
            self.out_data.y         = kf_y
            self.out_data.z         = kf_z
            self.out_data.angle_deg = -1    # TODO

            # Send data to pathfinding service
            out_conn.send( self.out_data )

            # Copy data to debug variable
            self.debug_data.x         = kf_x
            self.debug_data.y         = kf_y
            self.debug_data.z         = kf_z
            self.debug_data.angle_deg = -1    # TODO

            self.debug_data.r0   = anchors[0].r
            self.debug_data.r1   = anchors[1].r
            self.debug_data.r2   = anchors[2].r
            self.debug_data.r3   = anchors[3].r

            self.debug_data.phi0 = anchors[0].phi
            self.debug_data.phi1 = anchors[1].phi
            self.debug_data.phi2 = anchors[2].phi
            self.debug_data.phi3 = anchors[3].phi

            self.debug_data.los0 = anchors[0].los
            self.debug_data.los1 = anchors[1].los
            self.debug_data.los2 = anchors[2].los
            self.debug_data.los3 = anchors[3].los

            # self.debug_data.critical_anchor = critical_anchor

            # Signal debug endpoint thread to send data
            self.new_data_evt.set()


