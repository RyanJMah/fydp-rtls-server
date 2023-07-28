import sys
import time
import logs
import threading
import queue
import numpy as np
from dataclasses import dataclass
from typing import List
from app_mqtt import AnchorTelemetryData, IOS_TelemetryData
from mqtt_client import MqttClient, MqttMsg

from app_paths import AppPaths

from KalmanFilter import KalmanFilter as kf
# from kalman_helpers import initConstVelocityKF as initConstVel
from kalman_helpers import initConstAccelerationKF as initConstAcc

sys.path.append(AppPaths.TESTS_DIR)
sys.path.append(AppPaths.TESTS_DIR)

from visualize_iphone import run, push_coordinates

logger = logs.init_logger(__name__)

ANCHOR_0_COORDINATES = (615, 0, 273)
ANCHOR_1_COORDINATES = (615, 520, 263)
ANCHOR_2_COORDINATES = (0, 0, 277)
ANCHOR_3_COORDINATES = (123, 520, 263)

A = np.zeros((9,9))
H = np.zeros((3,9))
_kf = kf(A, H, 0)
A, B, H, Q, R, P_0, x_0 = initConstAcc()
_kf.assignSystemParameters(A, B, H, Q, R, P_0, x_0)

def trilateration(P1, P2, P3, r1, r2, r3):
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

def getQuad( x_coord, y_coord, anchor_x, anchor_y ):
    dx = anchor_x - x_coord
    dy = anchor_y - y_coord

    quadrant = 0

    if(dx > 0 and dy > 0):
        quadrant = 1
    if(dx < 0 and dy > 0):
        quadrant = 2
    if(dx < 0 and dy < 0):
        quadrant = 3
    if(dx > 0 and dy < 0):
        quadrant = 4
    
    return quadrant

def getQuadandRadius( x_coord, y_coord, anchor_x, anchor_y ):
    dx = anchor_x - x_coord
    dy = anchor_y - y_coord

    quadrant = 0
    radius = np.sqrt(dx**2 + dy**2)

    if(dx > 0 and dy > 0):
        quadrant = 1
    if(dx < 0 and dy > 0):
        quadrant = 2
    if(dx < 0 and dy < 0):
        quadrant = 3
    if(dx > 0 and dy < 0):
        quadrant = 4

    quad_dist = [quadrant, radius]
    
    return quad_dist

# def getUserAngle( y_coord, anchor_y, anchor_radius, anchor_quad, phone_norm_ang):

#     # TODO: Handle NaN anchor_radius
#     # TODO: Handle anchor_radius = 0

#     y_coord = np.abs(y_coord - anchor_y)
#     theta_deg = np.degrees(np.arcsin( (y_coord/anchor_radius) ))
#     phi = 0 # absolute angle 1

#     print(f"y_coord/anchor_radius = {y_coord/anchor_radius}")

#     if (anchor_quad == 1):
#         phi = theta_deg + phone_norm_ang
#     elif (anchor_quad == 2):
#         phi = 180 - theta_deg + phone_norm_ang
#     elif (anchor_quad == 3):
#         phi = 180 + theta_deg + phone_norm_ang
#     elif (anchor_quad == 4):
#         phi = 360 - theta_deg + phone_norm_ang

#     return phi

def getUserAngle(x_coord, y_coord, phone_norm_ang, anchor_x, anchor_y, anchor_quad):

    # TODO: Handle NaN anchor_radius
    # TODO: Handle anchor_radius = 0

    x_coord = np.abs(x_coord - anchor_x)
    y_coord = np.abs(y_coord - anchor_y)

    theta_deg = np.degrees(np.arctan( (y_coord/x_coord) ))

    phi = 0 # absolute angle

    if (anchor_quad == 1):
        phi = theta_deg + phone_norm_ang
    elif (anchor_quad == 2):
        phi = 180 - theta_deg + phone_norm_ang
    elif (anchor_quad == 3):
        phi = 180 + theta_deg + phone_norm_ang
    elif (anchor_quad == 4):
        phi = 360 - theta_deg + phone_norm_ang

    phi = theta_deg + phone_norm_ang

    return phi

def rollingMean(newData):
    global u
    u = np.array([u[1],u[2],u[3],u[4],newData])
    return np.mean(u)

def rollingSTD(newData):
    global std_dv
    std_dv = np.array([std_dv[1],std_dv[2],std_dv[3],std_dv[4],newData]);
    return np.std(std_dv)

class LowPassFilter:
    """
    First order low pass filter, discretized via trapazoidal rule,
    with a sample time of 5.5Hz
    """

    def __init__(self, tau: float):
        self.tau = tau

        self.y_n_minus_1 = 0
        self.u_n_minus_1 = 0

    def exec(self, u):
        y = ( 1/(11*self.tau + 1) ) * \
            (u + self.u_n_minus_1 - (1 - 11*self.tau)*self.y_n_minus_1)

        self.y_n_minus_1 = y
        self.u_n_minus_1 = u

        return y

class Anchor:
    def __init__(self):
        self.distance_cm = 0.0
        self.iphone_angle_degrees = 0.0
        self.iphone_angle_valid = False
        self.los = False

        self.distance_filter = LowPassFilter(0.075)
        self.angle_filter    = LowPassFilter(1)

class User:
    def __init__(self):
        self.x   = 0.0
        self.y   = 0.0

        self.phi = 0.0
        self.phi_filter = LowPassFilter(5)

        self.los_anchors: List[int] = []

g_anchors = [Anchor(), Anchor(), Anchor(), Anchor()]
g_user = User()

def anchor_data_handler(client: MqttClient, msg: MqttMsg, aid: int):
    global g_anchors

    data = AnchorTelemetryData.from_buffer_copy(msg.payload)

    if (data.status == 0):
        filtered_distance = g_anchors[aid].distance_filter.exec(data.distance_mm / 10)
        g_anchors[aid].distance_cm = filtered_distance

def ios_data_handler(client: MqttClient, msg: MqttMsg, uid: int, aid:int):
    global g_anchors

    data = IOS_TelemetryData.from_buffer_copy(msg.payload)

    g_anchors[aid].iphone_angle_valid = data.los

    filtered_angle = g_anchors[aid].angle_filter.exec(data.azimuth_deg)
    g_anchors[aid].iphone_angle_degrees = filtered_angle


# determine which anchors to use for trilateration based on LOS conditions
def select_anchors_for_trilateration():
    # (anchor3)            (anchor1)
    #                               
    #                               
    #                               
    # (anchor2)            (anchor0)

    global g_anchors

    los0 = g_anchors[0].iphone_angle_valid
    los1 = g_anchors[1].iphone_angle_valid
    los2 = g_anchors[2].iphone_angle_valid
    los3 = g_anchors[3].iphone_angle_valid

    los = [
        g_anchors[0].iphone_angle_valid,
        g_anchors[1].iphone_angle_valid,
        g_anchors[2].iphone_angle_valid,
        g_anchors[3].iphone_angle_valid
    ]

    r = [
        g_anchors[0].distance_cm,
        g_anchors[1].distance_cm,
        g_anchors[2].distance_cm,
        g_anchors[3].distance_cm
    ]

    phi = [
        g_anchors[0].iphone_angle_degrees,
        g_anchors[1].iphone_angle_degrees,
        g_anchors[2].iphone_angle_degrees,
        g_anchors[3].iphone_angle_degrees
    ]

    #################################################
    # DEFAULT VALUES
    trilat_a1 = ANCHOR_1_COORDINATES
    trilat_r1 = r[1]

    trilat_a2 = ANCHOR_3_COORDINATES
    trilat_r2 = r[3]

    trilat_a3 = ANCHOR_2_COORDINATES
    trilat_r3 = r[2]
    #################################################

    num_los = len([l for l in [los0, los1, los2, los3] if l])

    if num_los > 1:
        best_anchor = np.argmin(phi)

        for i in range(len(los)):
            if i != best_anchor:
                los[i] = False

    if los[0]:
        trilat_a1 = ANCHOR_2_COORDINATES
        trilat_r1 = r[2]

        trilat_a2 = ANCHOR_0_COORDINATES
        trilat_r2 = r[0]

        trilat_a3 = ANCHOR_1_COORDINATES
        trilat_r3 = r[1]

    elif los[1]:
        trilat_a1 = ANCHOR_0_COORDINATES
        trilat_r1 = r[0]

        trilat_a2 = ANCHOR_1_COORDINATES
        trilat_r2 = r[1]

        trilat_a3 = ANCHOR_2_COORDINATES
        trilat_r3 = r[2]

    elif los[2]:
        trilat_a1 = ANCHOR_3_COORDINATES
        trilat_r1 = r[3]

        trilat_a2 = ANCHOR_2_COORDINATES
        trilat_r2 = r[2]

        trilat_a3 = ANCHOR_0_COORDINATES
        trilat_r3 = r[0]

    elif los[3]:
        trilat_a1 = ANCHOR_1_COORDINATES
        trilat_r1 = r[1]

        trilat_a2 = ANCHOR_3_COORDINATES
        trilat_r2 = r[3]

        trilat_a3 = ANCHOR_2_COORDINATES
        trilat_r3 = r[2]

    return ( trilat_a1, trilat_a2, trilat_a3, \
             trilat_r1, trilat_r2, trilat_r3 )

def localization_thread():
    global g_anchors
    global g_user
    global _kf

    anchor0_coordinates = np.array([*ANCHOR_0_COORDINATES])
    anchor1_coordinates = np.array([*ANCHOR_1_COORDINATES])
    anchor2_coordinates = np.array([*ANCHOR_2_COORDINATES])
    anchor3_coordinates = np.array([*ANCHOR_3_COORDINATES])
    
    a0, a1, a2, a3 = g_anchors

    while (1):
        r0, phi0 = g_anchors[0].distance_cm, g_anchors[0].iphone_angle_degrees
        r1, phi1 = g_anchors[1].distance_cm, g_anchors[1].iphone_angle_degrees
        r2, phi2 = g_anchors[2].distance_cm, g_anchors[2].iphone_angle_degrees
        r3, phi3 = g_anchors[3].distance_cm, g_anchors[3].iphone_angle_degrees

        trilat_input = select_anchors_for_trilateration()

        coords, _ = trilateration(*trilat_input)
        x, y, z = coords
        
        '''
        *** DO NOT TOUCH ***
            KALMAN FILTER
        '''
        meas = np.array([x, y, z])
        meas.shape = (len(meas), 1)
        _kf.performKalmanFilter(np.array(meas), 0)
        kf_x = _kf.x_m[0]
        kf_y = _kf.x_m[1]
        '''
        *** DO NOT TOUCH ***
        '''

        userAngle_deg = 0
        quadrant = 0

        if( a0.iphone_angle_valid ):
            quadrant = getQuad( kf_x, kf_y,
                                anchor0_coordinates[0],
                                anchor0_coordinates[1] )

            userAngle_deg = getUserAngle( kf_x, kf_y, a0.iphone_angle_degrees,
                                          anchor0_coordinates[0],
                                          anchor0_coordinates[1],
                                          quadrant )

            # userAngle_deg = getUserAngle( y,
            #                               anchor0_coordinates[1],
            #                               a0.distance_cm,
            #                               quadrant,
            #                               a0.iphone_angle_degrees )

        elif( a1.iphone_angle_valid ):
            quadrant = getQuad( kf_x, kf_y,
                                anchor1_coordinates[0],
                                anchor1_coordinates[1] )

            userAngle_deg = getUserAngle( kf_x, kf_y, a1.iphone_angle_degrees,
                                          anchor1_coordinates[0],
                                          anchor1_coordinates[1],
                                          quadrant )

            # userAngle_deg = getUserAngle( y,
            #                               anchor1_coordinates[1],
            #                               a1.distance_cm,
            #                               quadrant,
            #                               a1.iphone_angle_degrees )

        elif( a2.iphone_angle_valid ):
            quadrant = getQuad( kf_x, kf_y,
                                anchor2_coordinates[0],
                                anchor2_coordinates[1] )

            userAngle_deg = getUserAngle( kf_x, kf_y, a2.iphone_angle_degrees,
                                          anchor2_coordinates[0],
                                          anchor2_coordinates[1],
                                          quadrant )

            # userAngle_deg = getUserAngle( y,
            #                               anchor2_coordinates[1],
            #                               a2.distance_cm,
            #                               quadrant,
            #                               a2.iphone_angle_degrees )

        elif ( a3.iphone_angle_valid ):
            quadrant = getQuad( kf_x, kf_y,
                                anchor3_coordinates[0],
                                anchor3_coordinates[1] )
            
            userAngle_deg = getUserAngle( kf_x, kf_y, a3.iphone_angle_degrees,
                                          anchor3_coordinates[0],
                                          anchor3_coordinates[1],
                                          quadrant )

        logger.warning(f"User angle: {userAngle_deg}")
        # logger.warning(f"Quandrant: {quadrant}")
        # logger.warning(f"r0 = {r0}, r1 = {r1}, r2 = {r2}")
        # logger.warning(f"phi0 = {phi0}, phi1 = {phi1}, phi2 = {phi2}, phi3 = ")
        # print()

        # logger.info(f"(x, y) = ({x}, {y})")

        # userAngle_deg = g_user.phi_filter.exec(userAngle_deg)

        push_coordinates( kf_x, kf_y, userAngle_deg,
                          r0, phi0,
                          r1, phi1,
                          r2, phi2,
                          r3, phi3,
                          los0,
                          los1,
                          los2,
                          los3 )

        time.sleep(0.1)

def main():
    client = MqttClient()

    client.connect("localhost", 1883)

    client.subscribe("gl/anchor/<id>/data", anchor_data_handler)
    client.subscribe("gl/user/<uid>/data/<aid>", ios_data_handler)

    client.start_mainloop()

    t = threading.Thread(target=localization_thread, daemon=True)
    t.start()

    run()

if __name__ == "__main__":
    main()
