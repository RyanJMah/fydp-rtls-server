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
from data_ingestion_service import DIS_OutData

from KalmanFilter import KalmanFilter as kf
from kalman_helpers import initConstVelocityKF as initConstVel
from kalman_helpers import initConstAccelerationKF as initConstAcc


logger = logs.init_logger(__name__)

HARDCODED_UID = 69

DEBUG_ENDPOINT_ADDRESS = "0.0.0.0"
DEBUG_ENDPOINT_PORT    = 6969

@dataclass
class _AnchorState:
    r: float
    phi: float
    los: bool

@dataclass
class _UserState:
    x: float
    y: float
    z: float
    angle_deg: float
    critical_anchor: int

@dataclass
class LocalizationService_DebugData:
    x: float
    y: float
    z: float
    angle_deg: float

    r0: float
    r1: float
    r2: float
    r3: float

    phi0: float
    phi1: float
    phi2: float
    phi3: float

    los0: bool
    los1: bool
    los2: bool
    los3: bool

    critical_anchor: int


@dataclass
class LocalizationService_OutData:
    uid       : int
    x         : float
    y         : float
    z         : float
    angle_deg : float


class LocalizationService(AbstractService):
    def __init__(self, in_conn: Any, out_conn: Any):
        super().__init__(in_conn, out_conn)

        self.kf: Any = None
        self.ANCHOR_COORDINATES: Optional[ Dict[int, Any] ] = None

        self.anchor: List[_AnchorState] = []
        self.user: Dict[int, _UserState] = {}


    def init_kalman_filter(self):
        A = np.zeros((6,6))
        H = np.zeros((3,6))

        self.kf = kf(A, H, 0)

        A, B, H, Q, R, P_0, x_0 = initConstVel()
        self.kf.assignSystemParameters(A, B, H, Q, R, P_0, x_0)


    def initialize(self):
        self.ANCHOR_COORDINATES = { i: a.get_coords() for i, a in GL_CONF.anchors.items() }

        self.anchor = [ _AnchorState(0, 0, False) for _ in range(GL_CONF.num_anchors) ]
        self.user   = { HARDCODED_UID: _UserState(0, 0, 0, 0, 0) }

        self.init_kalman_filter()


    def trilateration(self, a1, a2, a3, r1, r2, r3):
        P1 = self.ANCHOR_COORDINATES[a1]
        P2 = self.ANCHOR_COORDINATES[a2]
        P3 = self.ANCHOR_COORDINATES[a3]

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
    def select_anchors_for_trilateration(self):
        # (anchor3)            (anchor1)
        #                               
        #                               
        #                               
        # (anchor2)            (anchor0)

        los0 = self.anchor[0].los
        los1 = self.anchor[1].los
        los2 = self.anchor[2].los
        los3 = self.anchor[3].los

        los = [
            self.anchor[0].los,
            self.anchor[1].los,
            self.anchor[2].los,
            self.anchor[3].los
        ]

        r = [
            self.anchor[0].r,
            self.anchor[1].r,
            self.anchor[2].r,
            self.anchor[3].r
        ]

        phi = [
            self.anchor[0].phi,
            self.anchor[1].phi,
            self.anchor[2].phi,
            self.anchor[3].phi
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


    # this shouldn't exist, but it was how we did it before, idk
    # how not doing it like this will affect things, and I'm
    # just refactoring rn so...
    def ingest_data_thread(self):
        while (1):
            data = self.in_conn.recv()

            self.anchor[data.aid].r   = data.distance_cm
            self.anchor[data.aid].phi = data.angle_deg
            self.anchor[data.aid].los = data.los


    def debug_endpoint_thread(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # this is so we can restart the program without waiting for the socket to timeout
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        sock.bind( (DEBUG_ENDPOINT_ADDRESS, DEBUG_ENDPOINT_PORT) )
        sock.listen()

        conn, addr = sock.accept()
        with conn:
            logger.info("client connected to debug endpoint...")

            while (1):
                r0, phi0 = self.anchor[0].r, self.anchor[0].phi
                r1, phi1 = self.anchor[1].r, self.anchor[1].phi
                r2, phi2 = self.anchor[2].r, self.anchor[2].phi
                r3, phi3 = self.anchor[3].r, self.anchor[3].phi

                los0 = self.anchor[0].los
                los1 = self.anchor[1].los
                los2 = self.anchor[2].los
                los3 = self.anchor[3].los

                x               = self.user[HARDCODED_UID].x
                y               = self.user[HARDCODED_UID].y
                z               = self.user[HARDCODED_UID].z
                angle_deg       = self.user[HARDCODED_UID].angle_deg
                critical_anchor = self.user[HARDCODED_UID].critical_anchor

                debug_data = LocalizationService_DebugData( x, y, z, angle_deg, \
                                                            r0, r1, r2, r3, \
                                                            phi0, phi1, phi2, phi3, \
                                                            los0, los1, los2, los3, \
                                                            critical_anchor )
                conn.send( pickle.dumps(debug_data) )

                time.sleep(0.1)

    def main(self, in_conn, out_conn):
        assert(in_conn is not None)
        assert(out_conn is not None)

        self.initialize()

        threading.Thread(target=self.ingest_data_thread, daemon=True).start()

        if GL_CONF.loc_debug_endpoint:
            threading.Thread(target=self.debug_endpoint_thread, daemon=True).start()

        while (1):
            trilat_input    = self.select_anchors_for_trilateration()
            critical_anchor = trilat_input[1]

            coords, _ = self.trilateration( *trilat_input )

            x, y, z = coords

            # Run coords through Kalman filter
            meas = np.array([x, y, z])
            meas.shape = (len(meas), 1)
            self.kf.performKalmanFilter(np.array(meas), 0)

            kf_x = self.kf.x_m[0]
            kf_y = self.kf.x_m[1]
            kf_z = self.kf.x_m[2]

            self.user[HARDCODED_UID].x = kf_x
            self.user[HARDCODED_UID].y = kf_y
            self.user[HARDCODED_UID].z = kf_z
            self.user[HARDCODED_UID].angle_deg = -1
            self.user[HARDCODED_UID].critical_anchor = critical_anchor

            # out_data = LocalizationService_OutData( HARDCODED_UID, kf_x, kf_y, kf_z, -1 )
            # out_conn.send(out_data)

            time.sleep(0.1)

