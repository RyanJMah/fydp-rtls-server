import queue
from gl_conf import GL_CONF
from lpf import LowPassFilter
from abstract_service import AbstractService

DISTANCE_FILTER_TIME_CONSTANT = 0.175
ANGLE_FILTER_TIME_CONSTANT    = 1

class _AnchorState:
    def __init__(self):
        self.distance_filter = LowPassFilter(DISTANCE_FILTER_TIME_CONSTANT)
        self.angle_filter    = LowPassFilter(ANGLE_FILTER_TIME_CONSTANT)

class LocalizationService(AbstractService):
    def __init__(self, in_queue: queue.Queue, out_queue: queue.Queue):
        super().__init__(in_queue, out_queue)

        self.anchor_states = [ _AnchorState() for _ in range(GL_CONF.num_anchors) ]

    def trilateration(self, a1, a2, a3, r1, r2, r3):
        P1 = ANCHOR_COORDINATES[a1]
        P2 = ANCHOR_COORDINATES[a2]
        P3 = ANCHOR_COORDINATES[a3]

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

    def mainloop(self):
        while (1):
            pass
