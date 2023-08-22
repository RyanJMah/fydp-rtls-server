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
from least_squares import gauss_newton_lse, linearized_lse
from abstract_service import AbstractService
from data_ingestion_service import DIS_OutData, AnchorRangingState

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
    w0: float = 0
    w1: float = 1 / 3
    w2: float = 1 / 3
    w3: float = 1 / 3

    gn_iters: int = 0

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

class LocalizationService(AbstractService):
    def init_kalman_filter(self):
        A = np.zeros((9,9))
        H = np.zeros((3,9))

        self.kf = kf(A, H, 0)

        # A, B, H, Q, R, P_0, x_0 = initConstVel()
        A, B, H, Q, R, P_0, x_0 = initConstAcc()

        self.kf.assignSystemParameters(A, B, H, Q, R, P_0, x_0)


    def init_gauss_newton_params(self):
        """
        Want to minimize the sum of squared errors between the measured and estimated ranges

        Reference:
            - https://en.wikipedia.org/wiki/Gauss–Newton_algorithm

        Refer to ./tools/math.ipynb for derivation of the residual function and its Jacobian
        """

        # Residual function
        def r(x: NDArray) -> NDArray:
            # Remeber: Minimizing the square of a positive function is equivalent
            #          to minimizing the function itself...

            x_ = x[0][0]
            y_ = x[1][0]
            z_ = x[2][0]

            r0 = self.loc_state.r0
            r1 = self.loc_state.r1
            r2 = self.loc_state.r2
            r3 = self.loc_state.r3

            w0 = self.loc_state.w0
            w1 = self.loc_state.w1
            w2 = self.loc_state.w2
            w3 = self.loc_state.w3

            return np.array([
                [ np.sqrt(w0)*np.abs( ( (X0 - x_)**2 + (Y0 - y_)**2 + (Z0 - z_)**2 ) - r0**2 ) ],
                [ np.sqrt(w1)*np.abs( ( (X1 - x_)**2 + (Y1 - y_)**2 + (Z1 - z_)**2 ) - r1**2 ) ],
                [ np.sqrt(w2)*np.abs( ( (X2 - x_)**2 + (Y2 - y_)**2 + (Z2 - z_)**2 ) - r2**2 ) ],
                [ np.sqrt(w3)*np.abs( ( (X3 - x_)**2 + (Y3 - y_)**2 + (Z3 - z_)**2 ) - r3**2 ) ]
            ])

        # Jacobian of residual function, derived via sympy in ./tools/math.ipynb
        def J(x: NDArray) -> NDArray:
            x_ = x[0][0]
            y_ = x[1][0]
            z_ = x[2][0]

            r0 = self.loc_state.r0
            r1 = self.loc_state.r1
            r2 = self.loc_state.r2
            r3 = self.loc_state.r3

            w0 = self.loc_state.w0
            w1 = self.loc_state.w1
            w2 = self.loc_state.w2
            w3 = self.loc_state.w3

            # sign of the error
            sign0 = np.sign( ( (X0 - x_)**2 + (Y0 - y_)**2 + (Z0 - z_)**2 ) - r0**2 )
            sign1 = np.sign( ( (X1 - x_)**2 + (Y1 - y_)**2 + (Z1 - z_)**2 ) - r1**2 )
            sign2 = np.sign( ( (X2 - x_)**2 + (Y2 - y_)**2 + (Z2 - z_)**2 ) - r2**2 )
            sign3 = np.sign( ( (X3 - x_)**2 + (Y3 - y_)**2 + (Z3 - z_)**2 ) - r3**2 )

            return np.array([
                [ 2*np.sqrt(w0)*(x_ - X0)*sign0, 2*np.sqrt(w0)*(y_ - Y0)*sign0, 2*np.sqrt(w0)*(z_ - Z0)*sign0 ],
                [ 2*np.sqrt(w1)*(x_ - X1)*sign1, 2*np.sqrt(w1)*(y_ - Y1)*sign1, 2*np.sqrt(w1)*(z_ - Z1)*sign1 ],
                [ 2*np.sqrt(w2)*(x_ - X2)*sign2, 2*np.sqrt(w2)*(y_ - Y2)*sign2, 2*np.sqrt(w2)*(z_ - Z2)*sign2 ],
                [ 2*np.sqrt(w3)*(x_ - X3)*sign3, 2*np.sqrt(w3)*(y_ - Y3)*sign3, 2*np.sqrt(w3)*(z_ - Z3)*sign3 ]
            ])

        self.gn_r         = r
        self.gn_J         = J
        self.gn_n         = 3   # 3 unknowns: x, y, z
        self.gn_m         = 4   # 4 equations: 4 anchors
        self.gn_tol       = 0.5
        self.gn_max_iters = GL_CONF.loc_service_max_gauss_newton_iterations


    def initialize(self):
        # init "global" user location, which is periodically sent to pathfinding service
        self.out_data = LocalizationService_OutData()

        # init "global" localization state
        self.loc_state = LocalizationService_State()

        # event that fires when new data is available, used to signal to debug endpoint
        self.new_data_evt = threading.Event()

        self.init_kalman_filter()
        self.init_gauss_newton_params()


    def choose_lse_A_b(self, anchor_num: int) -> Tuple[NDArray, NDArray]:
        # The anchor at which we linearize the LSE problem around is very important.
        #
        # If we linearize around an unreliable anchor, the gauss-newton solver will
        # probably not converge.
        #
        # We will setup matrices for linearized LSE around each anchor, and then
        # choose the one with the highest weight to linearize around.

        r0 = self.loc_state.r0
        r1 = self.loc_state.r1
        r2 = self.loc_state.r2
        r3 = self.loc_state.r3

        A: NDArray
        b: NDArray

        # Anchor 0
        if anchor_num == 0:
            A = np.array([
                [X1-X0, Y1-Y0, Z1-Z0],
                [X2-X0, Y2-Y0, Z2-Z0],
                [X3-X0, Y3-Y0, Z3-Z0]
            ])
            b  = 0.5 * np.array([
                [r0**2 - r1**2 + (X1**2 + Y1**2 + Z1**2) - (X0**2 + Y0**2 + Z0**2)],
                [r0**2 - r2**2 + (X2**2 + Y2**2 + Z2**2) - (X0**2 + Y0**2 + Z0**2)],
                [r0**2 - r3**2 + (X3**2 + Y3**2 + Z3**2) - (X0**2 + Y0**2 + Z0**2)]
            ])

        # Anchor 1
        elif anchor_num == 1:
            A = np.array([
                [X0-X1, Y0-Y1, Z0-Z1],
                [X2-X1, Y2-Y1, Z2-Z1],
                [X3-X1, Y3-Y1, Z3-Z1]
            ])
            b  = 0.5 * np.array([
                [r1**2 - r0**2 + (X0**2 + Y0**2 + Z0**2) - (X1**2 + Y1**2 + Z1**2)],
                [r1**2 - r2**2 + (X2**2 + Y2**2 + Z2**2) - (X1**2 + Y1**2 + Z1**2)],
                [r1**2 - r3**2 + (X3**2 + Y3**2 + Z3**2) - (X1**2 + Y1**2 + Z1**2)]
            ])

        # Anchor 2
        elif anchor_num == 2:
            A = np.array([
                [X0-X2, Y0-Y2, Z0-Z2],
                [X1-X2, Y1-Y2, Z1-Z2],
                [X3-X2, Y3-Y2, Z3-Z2]
            ])
            b  = 0.5 * np.array([
                [r2**2 - r0**2 + (X0**2 + Y0**2 + Z0**2) - (X2**2 + Y2**2 + Z2**2)],
                [r2**2 - r1**2 + (X1**2 + Y1**2 + Z1**2) - (X2**2 + Y2**2 + Z2**2)],
                [r2**2 - r3**2 + (X3**2 + Y3**2 + Z3**2) - (X2**2 + Y2**2 + Z2**2)]
            ])

        # Anchor 3
        elif anchor_num == 3:
            A = np.array([
                [X0-X3, Y0-Y3, Z0-Z3],
                [X1-X3, Y1-Y3, Z1-Z3],
                [X2-X3, Y2-Y3, Z2-Z3]
            ])
            b  = 0.5 * np.array([
                [r3**2 - r0**2 + (X0**2 + Y0**2 + Z0**2) - (X3**2 + Y3**2 + Z3**2)],
                [r3**2 - r1**2 + (X1**2 + Y1**2 + Z1**2) - (X3**2 + Y3**2 + Z3**2)],
                [r3**2 - r2**2 + (X2**2 + Y2**2 + Z2**2) - (X3**2 + Y3**2 + Z3**2)]
            ])

        else:
            logger.error("Invalid anchor index: {}".format(best_anchor))

        return A, b

    def multilateration(self, anchors: Dict[int, AnchorRangingState]):
        """
        Multilateration via weighted least squares estimation
            - Reference equation 5 of https://ieeexplore.ieee.org/document/8911811
        """

        # First, try using the last known location as the initial guess, see if it converges
        x0_ = np.array([
            [self.loc_state.x],
            [self.loc_state.y],
            [self.loc_state.z]
        ])
        iters, x = gauss_newton_lse( self.gn_r,
                                     self.gn_J,
                                     x0_,
                                     self.gn_n,
                                     self.gn_m,
                                     tolerance=self.gn_tol,
                                     max_iters=self.gn_max_iters )

        if iters < self.gn_max_iters:
            # Update "global" state
            self.loc_state.gn_iters = iters
            self.loc_state.critical_anchor = -1

            return x[0][0], x[1][0], x[2][0]

        # If Gauss-Newton fails to converge, try linearized LSE as the initial guess
        #
        # Try Gauss-Newton with the initial guess being linearized around each anchor, try
        # the anchor with the highest weight first
        #
        # If Gauss-Newton fails to converge, try the next highest weight anchor
        #
        # If none of the anchors work, use the linearized estimate, linearized around the
        # anchor with the highest weight, as the fallback estimate

        weights = [self.loc_state.w0, self.loc_state.w1, self.loc_state.w2, self.loc_state.w3]

        fallback_x: NDArray

        for i in range(GL_CONF.num_anchors):
            # Choose A and b based on the anchor with the highest weight
            best_anchor = np.argmax( weights )

            A, b = self.choose_lse_A_b( best_anchor )

            # Use linearized, non-weighted least squares estimate as initial guess
            x0_ = linearized_lse(A, b)

            if i == 0:
                # Use the first try as the fallback estimate
                fallback_x = x0_

            # Use Gauss-Newton to refine estimate with weights
            start = time.time()
            iters, x = gauss_newton_lse( self.gn_r,
                                         self.gn_J,
                                         x0_,
                                         self.gn_n,
                                         self.gn_m,
                                         tolerance=self.gn_tol,
                                         max_iters=self.gn_max_iters )
            end = time.time()


            if iters == self.gn_max_iters:  # Fails to converge
                # Try the next highest weight anchor
                weights[best_anchor] = 0
                continue

            else:
                if iters == 0:
                    logger.warning("Gauss-Newton converged in 0 iterations, something might be wrong...")

                logger.info(f"gauss-newton iterations: {iters}, time: {end - start}, linearized around anchor {best_anchor}")

                # Update "global" state
                self.loc_state.gn_iters = iters
                self.loc_state.critical_anchor = best_anchor

                return x[0][0], x[1][0], x[2][0]


        logger.warning("Gauss-Newton did not converge, falling back to linearized least squares estimate...")

        # Update "global" state
        self.loc_state.gn_iters = self.gn_max_iters
        self.loc_state.critical_anchor = -1

        return fallback_x[0][0], fallback_x[1][0], fallback_x[2][0]


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

                conn.send( pickle.dumps(self.loc_state) )


    def main(self, in_conn, out_conn):
        assert(in_conn is not None)
        assert(out_conn is not None)

        self.initialize()

        logger.info("Starting localization service...")

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

            # self.loc_state.critical_anchor = critical_anchor

            # Signal debug endpoint thread to send data
            self.new_data_evt.set()


