import time
import socket
import pickle
import numpy as np
import threading
import atexit
from typing import Optional, List, Dict, Tuple, Any
from numpy.typing import NDArray
from dataclasses import dataclass

import logs
from gl_conf import GL_CONF
from lpf import LowPassFilter
from least_squares import weighted_linearized_lse, linearized_lse
from abstract_service import AbstractService
from data_ingestion_service import DIS_OutData, AnchorRangingState
from debug_endpoint_service import DebugEndpointService, DebugEndpointData

from KalmanFilter import KalmanFilter as kf
from KalmanFilter import initConstVelocityKF as initConstVel
from KalmanFilter import initConstAccelerationKF as initConstAcc

logger = logs.init_logger(__name__)

@dataclass
class LocalizationService_State:
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

    # anchor weights
    w0: float = 0.0
    w1: float = 0.0
    w2: float = 0.0
    w3: float = 0.0

    critical_anchor: int = 0


@dataclass
class LocalizationService_OutData:
    x         : float = 0.0
    y         : float = 0.0
    z         : float = 0.0
    angle_deg : float = 0.0


# global constants for convenience
X0 = GL_CONF.anchor_coords[0][0]
Y0 = GL_CONF.anchor_coords[0][1]
Z0 = GL_CONF.anchor_coords[0][2]

X1 = GL_CONF.anchor_coords[1][0]
Y1 = GL_CONF.anchor_coords[1][1]
Z1 = GL_CONF.anchor_coords[1][2]

X2 = GL_CONF.anchor_coords[2][0]
Y2 = GL_CONF.anchor_coords[2][1]
Z2 = GL_CONF.anchor_coords[2][2]

X3 = GL_CONF.anchor_coords[3][0]
Y3 = GL_CONF.anchor_coords[3][1]
Z3 = GL_CONF.anchor_coords[3][2]

R0_SQUARED = X0**2 + Y0**2 + Z0**2
R1_SQUARED = X1**2 + Y1**2 + Z1**2
R2_SQUARED = X2**2 + Y2**2 + Z2**2
R3_SQUARED = X3**2 + Y3**2 + Z3**2


class LocalizationService(AbstractService):
    def init_kalman_filter(self):
        A = np.zeros((9,9))
        H = np.zeros((3,9))

        self.kf = kf(A, H, 0)

        # A, B, H, Q, R, P_0, x_0 = initConstVel()
        A, B, H, Q, R, P_0, x_0 = initConstAcc()

        self.kf.assignSystemParameters(A, B, H, Q, R, P_0, x_0)


    def precompute_lse_A_matrix(self):
        # The A matrix is constant, so we can precompute it at initialization time.
        #
        # Reference equation (5) in https://ieeexplore.ieee.org/document/8911811

        self.lse_A: Dict[int, NDArray] = {}
        self.lse_W: Dict[int, NDArray] = {}

        # Anchor 0
        self.lse_A[0] = np.array([
                            [X1-X0, Y1-Y0, Z1-Z0],
                            [X2-X0, Y2-Y0, Z2-Z0],
                            [X3-X0, Y3-Y0, Z3-Z0]  ])

        # Anchor 1
        self.lse_A[1] = np.array([
                            [X0-X1, Y0-Y1, Z0-Z1],
                            [X2-X1, Y2-Y1, Z2-Z1],
                            [X3-X1, Y3-Y1, Z3-Z1]  ])

        # Anchor 2
        self.lse_A[2] = np.array([
                            [X0-X2, Y0-Y2, Z0-Z2],
                            [X1-X2, Y1-Y2, Z1-Z2],
                            [X3-X2, Y3-Y2, Z3-Z2]  ])

        # Anchor 3
        self.lse_A[3] = np.array([
                            [X0-X3, Y0-Y3, Z0-Z3],
                            [X1-X3, Y1-Y3, Z1-Z3],
                            [X2-X3, Y2-Y3, Z2-Z3]  ])


    def initialize(self):
        # init "global" user location, which is periodically sent to pathfinding service
        self.out_data = LocalizationService_OutData()

        # init "global" localization state
        self.loc_state = LocalizationService_State()

        self.init_kalman_filter()
        self.precompute_lse_A_matrix()


    def select_critical_anchor(self):
        # hardcoded for now, want to assign based on NLOS probabilities in the future
        self.loc_state.critical_anchor = 2

    
    def assign_anchor_weights(self):
        # hardcoded for now, want to assign based on NLOS probabilities in the future

        self.loc_state.w0 = 1
        self.loc_state.w1 = 1
        self.loc_state.w2 = 1
        self.loc_state.w3 = 1


    def choose_lse_params(self) -> Tuple[NDArray, NDArray, NDArray]:
        """
        Returns A, b, and W for weighted, linearized LSE problem.

        Reference equation (5) in https://ieeexplore.ieee.org/document/8911811
        """

        # The anchor at which we linearize the LSE problem around is very important.
        #
        # The anchor that we linearize around should be the most reliable anchor. We
        # call this the "critical anchor". The critical anchor is the anchor that has
        # the lowest NLOS probability.
        #
        # The critical anchor implicitly has the highest weight, since we are basing
        # the linearization around it.
        #
        # The other anchors have weights that show up in the diagonal elements of W.

        r0 = self.loc_state.r0
        r1 = self.loc_state.r1
        r2 = self.loc_state.r2
        r3 = self.loc_state.r3

        A = self.lse_A[self.loc_state.critical_anchor]

        b: NDArray
        W: NDArray

        if self.loc_state.critical_anchor == 0:
            b = 0.5 * np.array([
                            [r0**2 - r1**2 + R1_SQUARED - R0_SQUARED],
                            [r0**2 - r2**2 + R2_SQUARED - R0_SQUARED],
                            [r0**2 - r3**2 + R3_SQUARED - R0_SQUARED]  ])

            # W = np.diag([self.loc_state.w1, self.loc_state.w2, self.loc_state.w3])
            W = np.diag([np.sqrt(self.loc_state.w1), np.sqrt(self.loc_state.w2), np.sqrt(self.loc_state.w3)])

        # Anchor 1
        elif self.loc_state.critical_anchor == 1:
            b = 0.5 * np.array([
                            [r1**2 - r0**2 + R0_SQUARED - R1_SQUARED],
                            [r1**2 - r2**2 + R2_SQUARED - R1_SQUARED],
                            [r1**2 - r3**2 + R3_SQUARED - R1_SQUARED]  ])

            # W = np.diag([self.loc_state.w0, self.loc_state.w2, self.loc_state.w3])
            W = np.diag([np.sqrt(self.loc_state.w0), np.sqrt(self.loc_state.w2), np.sqrt(self.loc_state.w3)])

        # Anchor 2
        elif self.loc_state.critical_anchor == 2:
            b = 0.5 * np.array([
                            [r2**2 - r0**2 + R0_SQUARED - R2_SQUARED],
                            [r2**2 - r1**2 + R1_SQUARED - R2_SQUARED],
                            [r2**2 - r3**2 + R3_SQUARED - R2_SQUARED]  ])

            # W = np.diag([self.loc_state.w0, self.loc_state.w1, self.loc_state.w3])
            W = np.diag([np.sqrt(self.loc_state.w0), np.sqrt(self.loc_state.w1), np.sqrt(self.loc_state.w3)])

        # Anchor 3
        elif self.loc_state.critical_anchor == 3:
            b = 0.5 * np.array([
                            [r3**2 - r0**2 + R0_SQUARED - R3_SQUARED],
                            [r3**2 - r1**2 + R1_SQUARED - R3_SQUARED],
                            [r3**2 - r2**2 + R2_SQUARED - R3_SQUARED]  ])

            # W = np.diag([self.loc_state.w0, self.loc_state.w1, self.loc_state.w2])
            W = np.diag([np.sqrt(self.loc_state.w0), np.sqrt(self.loc_state.w1), np.sqrt(self.loc_state.w2)])

        else:
            logger.error("Invalid anchor index: {}".format(self.loc_state.critical_anchor))

        return A, b, W


    def multilateration(self) -> Tuple[float, float, float]:
        """
        Multilateration via weighted least squares estimation
            - Reference equation 5 of https://ieeexplore.ieee.org/document/8911811

        Returns (x, y, z)
        """

        weights = [self.loc_state.w0, self.loc_state.w1, self.loc_state.w2, self.loc_state.w3]

        non_zero_weighted_anchors = [i for i in range(len(weights)) if weights[i] != 0]
        num_zero_weights          = len(weights) - len(non_zero_weighted_anchors)

        if num_zero_weights == 1:
            # For 4 anchors, fall back to trilateration if any of the weights are 0
            distances = [self.loc_state.r0, self.loc_state.r1, self.loc_state.r2, self.loc_state.r3]

            distances_for_trilat = [distances[i] for i in range(len(distances)) if weights[i] != 0]

            coords, _ = self.trilateration(*non_zero_weighted_anchors, *distances_for_trilat)

            return coords[0], coords[1], np.abs(coords[2])

        elif num_zero_weights > 1:
            # not enough anchors to solve for position
            logger.critical("Not enough anchors to solve for position!!")
            return


        A, b, W = self.choose_lse_params()

        # solve weighted, linearized least squares problem
        x = weighted_linearized_lse(A, b, W)
        # x = linearized_lse(A, b)

        return x[0][0], x[1][0], x[2][0]


    def trilateration(self, a1, a2, a3, r1, r2, r3):
        P1 = GL_CONF.anchor_coords[a1]
        P2 = GL_CONF.anchor_coords[a2]
        P3 = GL_CONF.anchor_coords[a3]

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


    def main(self, in_conn, out_conn):
        assert(in_conn is not None)
        assert(out_conn is not None)

        self.initialize()

        logger.info("Starting localization service...")

        while (1):
            anchor_data: DIS_OutData = self.in_conn.recv()     # type: ignore

            anchors = anchor_data.anchors

            # Assign to local state
            self.loc_state.r0   = anchors[0].r
            self.loc_state.r1   = anchors[1].r
            self.loc_state.r2   = anchors[2].r
            self.loc_state.r3   = anchors[3].r

            self.loc_state.phi0 = anchors[0].phi
            self.loc_state.phi1 = anchors[1].phi
            self.loc_state.phi2 = anchors[2].phi
            self.loc_state.phi3 = anchors[3].phi

            self.loc_state.los0 = anchors[0].los
            self.loc_state.los1 = anchors[1].los
            self.loc_state.los2 = anchors[2].los
            self.loc_state.los3 = anchors[3].los

            self.select_critical_anchor()
            self.assign_anchor_weights()

            x, y, z = self.multilateration()

            # Run coords through Kalman filter
            meas = np.array([x, y, z])
            meas.shape = (len(meas), 1)
            self.kf.performKalmanFilter(np.array(meas), 0)

            kf_x = self.kf.x_m[0][0]
            kf_y = self.kf.x_m[1][0]
            kf_z = self.kf.x_m[2][0]

            # Copy data to output variable
            self.out_data.x         = kf_x
            self.out_data.y         = kf_y
            self.out_data.z         = kf_z
            self.out_data.angle_deg = -1    # TODO

            # Send data to pathfinding service
            out_conn.send( self.out_data )

            # Copy data to debug variable
            self.loc_state.x         = kf_x
            self.loc_state.y         = kf_y
            self.loc_state.z         = kf_z
            self.loc_state.angle_deg = -1    # TODO

            # Push data to debug endpoint
            debug_endpoint_data = DebugEndpointData( tag="loc_state",
                                                     data=self.loc_state )

            DebugEndpointService.push(debug_endpoint_data)
